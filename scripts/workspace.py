#!/usr/bin/env python3
# pip3 install numpy scipy trimesh

"""
Calculate arm workspace
"""

import argparse
import numpy as np
from scipy import ndimage

from urdf_parser_py.urdf import URDF
import kdl_parser_py.urdf as kdl_urdf
import PyKDL as kdl

from scipy.spatial import ConvexHull
import trimesh

def get_chain_and_limits(robot: URDF, base_link: str, tip_link: str):
    ok, tree = kdl_urdf.treeFromUrdfModel(robot)
    if not ok:
        raise RuntimeError("Failed to build KDL tree from URDF")

    chain = tree.getChain(base_link, tip_link)

    # Extract joints in chain order + limits
    joint_names = []
    lower = []
    upper = []

    # KDL chain segments contain joint names; map them back to URDF joints for limits
    urdf_joint_map = {j.name: j for j in robot.joints}

    for i in range(chain.getNrOfSegments()):
        seg = chain.getSegment(i)
        j = seg.getJoint()
        if j.getType() == kdl.Joint.None:
            continue
        name = j.getName()
        joint_names.append(name)

        uj = urdf_joint_map.get(name, None)
        if uj is None or uj.limit is None:
            raise RuntimeError(f"No limits found for joint '{name}' in URDF")

        # Continuous joints: sample [-pi, pi] unless you want a different range
        if uj.type == "continuous":
            lo, hi = -np.pi, np.pi
        else:
            lo, hi = float(uj.limit.lower), float(uj.limit.upper)

        lower.append(lo)
        upper.append(hi)

    if len(joint_names) != chain.getNrOfJoints():
        # This *usually* matches; if not, something weird in the URDF/joint naming.
        raise RuntimeError("KDL joint count mismatch; check URDF joint names and chain.")

    return chain, joint_names, np.array(lower), np.array(upper)


def fk_points(chain: kdl.Chain, q_samples: np.ndarray):
    fk = kdl.ChainFkSolverPos_recursive(chain)
    pts = np.zeros((q_samples.shape[0], 3), dtype=np.float64)

    q_kdl = kdl.JntArray(chain.getNrOfJoints())
    out_frame = kdl.Frame()

    for i in range(q_samples.shape[0]):
        for j in range(chain.getNrOfJoints()):
            q_kdl[j] = float(q_samples[i, j])

        rc = fk.JntToCart(q_kdl, out_frame)
        if rc < 0:
            # Rare; skip if FK fails
            pts[i, :] = np.nan
            continue

        p = out_frame.p
        pts[i, :] = (p[0], p[1], p[2])

    pts = pts[~np.isnan(pts).any(axis=1)]
    return pts

def points_to_volume_mesh_stl(
    pts: np.ndarray,
    out_stl: str,
    voxel_size: float = 0.01,   # meters; try 0.005..0.02
    padding: int = 3,           # voxels around the bounding box
    closing_iters: int = 2,     # 0 to disable; try 1..4
    min_component_volume: float = 0.0,  # m^3, optional filtering
):
    """
    Convert point cloud to a non-convex volume mesh via voxel occupancy + marching cubes.
    """

    if pts.shape[0] < 10:
        raise ValueError("Need more points.")

    # Build occupancy grid
    pmin = pts.min(axis=0)
    pmax = pts.max(axis=0)

    # Grid origin (world coords) padded
    origin = pmin - padding * voxel_size
    dims = np.ceil((pmax - pmin) / voxel_size).astype(int) + 2 * padding + 1
    dims = np.maximum(dims, 1)

    # Map points to voxel indices
    ijk = np.floor((pts - origin) / voxel_size).astype(int)
    valid = np.all((ijk >= 0) & (ijk < dims), axis=1)
    ijk = ijk[valid]

    num_voxels = int(dims[0] * dims[1] * dims[2])
    if num_voxels > 5e7:  # ~50 million voxels ≈ 50 MB bool array
        raise RuntimeError(
            f"Voxel grid too large: {dims} ({num_voxels} voxels). "
            "Increase voxel_size."
        )

    occ = np.zeros(dims, dtype=bool)
    occ[ijk[:, 0], ijk[:, 1], ijk[:, 2]] = True

    # Morphological closing to fill gaps from sparse sampling ---
    if closing_iters > 0:
        # a small spherical-ish structuring element
        structure = ndimage.generate_binary_structure(3, 1)  # 6-connected
        occ = ndimage.binary_closing(occ, structure=structure, iterations=closing_iters)

    # Convert to Mesh with Marching Cubes
    mesh = trimesh.voxel.ops.matrix_to_marching_cubes(occ, pitch=voxel_size)

    # Put mesh into world coordinates (since marching cubes assumes grid origin at (0,0,0))
    mesh.apply_translation(origin)

    # Remove tiny disconnected components
    if min_component_volume > 0:
        comps = mesh.split(only_watertight=False)
        keep = []
        for c in comps:
            # If watertight, volume is meaningful; if not, we can fallback to bbox volume
            vol = c.volume if c.is_watertight else np.prod(c.bounds[1] - c.bounds[0])
            if vol >= min_component_volume:
                keep.append(c)
        if len(keep) == 0:
            raise RuntimeError("All components filtered out; lower min_component_volume.")
        mesh = trimesh.util.concatenate(keep)

    mesh.export(out_stl)
    return mesh

def main():
    # Process arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True, help="Path to URDF file (robot.urdf)")
    ap.add_argument("--base", required=True, help="Base link name")
    ap.add_argument("--tip", required=True, help="Tip/end-effector link name")
    ap.add_argument("--samples", type=int, default=200000, help="Number of random joint samples")
    ap.add_argument("--seed", type=int, default=0, help="RNG seed")
    ap.add_argument("--voxel_size", type=float, default=0.1, help="Voxel size in meters for volume mesh")
    ap.add_argument("--closing_iters", type=int, default=2, help="Morphological closing iterations for volume mesh")
    ap.add_argument("--min_component_volume", type=float, default=0.0, help="Minimum volume (m^3) to keep a mesh component")
    ap.add_argument("--hull_out", default="workspace_convex_hull.stl", help="Output STL path for convex hull")
    ap.add_argument("--mesh_out", default="workspace_volume.stl", help="Output STL path")
    args = ap.parse_args()

    # Load Model
    robot = URDF.from_xml_file(args.urdf)
    chain, joint_names, lo, hi = get_chain_and_limits(robot, args.base, args.tip)

    # Sample
    rng = np.random.default_rng(args.seed)
    q = rng.uniform(lo, hi, size=(args.samples, len(joint_names)))

    pts = fk_points(chain, q)
    if pts.shape[0] < 4:
        raise RuntimeError("Not enough valid FK points to build a 3D hull")
    
    print(f"Joints ({len(joint_names)}): {joint_names}")
    print(f"Points used: {len(pts)} / {args.samples}")

    # Export Convex Hull
    hull = ConvexHull(pts)
    mesh = trimesh.Trimesh(
        vertices=pts[hull.vertices],
        faces=hull.simplices,
        process=True
    )

    mesh.export(args.hull_out)

    print(f"Wrote Convex Hull: {args.hull_out}")

    # Export Mesh
    mesh = points_to_volume_mesh_stl(
        pts,
        out_stl=args.mesh_out,
        voxel_size=args.voxel_size,
        closing_iters=args.closing_iters,
        min_component_volume=args.min_component_volume,
    )

    print(f"Wrote workspace volume mesh: {args.mesh_out}")

if __name__ == "__main__":
    main()