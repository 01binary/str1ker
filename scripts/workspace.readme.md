# Calculate Robot Arm Workspace

A Python script that computes the reachable workspace of a robot arm from a URDF model.

It generates:

* A **convex hull** of the reachable tip positions
* A **non-convex volumetric mesh** using voxelization + marching cubes

Designed for:

* Ubuntu 20.04
* ROS Noetic
* Python 3

---

## Dependencies

ROS packages (usually already available in Noetic):

* `urdf_parser_py`
* `kdl_parser_py`
* `PyKDL`

Python packages:

```bash
pip3 install numpy scipy trimesh scikit-image
```

---

## How It Works

1. Loads URDF
2. Builds KDL kinematic chain
3. Extracts joint limits
4. Randomly samples joint space
5. Computes forward kinematics
6. Builds:

   * Convex hull mesh
   * Non-convex volume mesh

---

## Usage

```bash
python3 workspace.py \
  --urdf /description/robot.urdf \
  --base base_link \
  --tip tip_link \
  --samples 300000 \
  --seed 0 \
  --voxel_size 0.01 \
  --closing_iters 2 \
  --min_component_volume 0 \
  --hull_out workspace-hull.stl \
  --mesh_out workspace-mesh.stl
```

---

## Parameters

| Parameter                | Description                                                        |
| ------------------------ | ------------------------------------------------------------------ |
| `--urdf`                 | Path to URDF file                                                  |
| `--base`                 | Base link name                                                     |
| `--tip`                  | End-effector (tip) link name                                       |
| `--samples`              | Number of random joint samples                                     |
| `--seed`                 | RNG seed for reproducibility                                       |
| `--voxel_size`           | Voxel resolution in meters (recommended: 0.005–0.02)               |
| `--closing_iters`        | Morphological closing iterations (fills gaps from sparse sampling) |
| `--min_component_volume` | Remove mesh components smaller than this volume (m³)               |
| `--hull_out`             | Output STL path for convex hull                                    |
| `--mesh_out`             | Output STL path for non-convex volume mesh                         |

---

## Recommended Settings

For a ~1 meter reach arm:

| Quality       | Samples | Voxel Size |
| ------------- | ------- | ---------- |
| Fast preview  | 50k     | 0.02       |
| Balanced      | 200k    | 0.01       |
| High fidelity | 400k+   | 0.005      |

⚠ Very small voxel sizes can require large amounts of RAM.

---

## Output Files

* `workspace-hull.stl` — convex hull (fast, bounding volume)
* `workspace-mesh.stl` — non-convex volumetric workspace

Both can be imported into CAD tools like:

* SolidWorks
* Fusion 360
* Blender
* FreeCAD

---

## Notes

* This computes **geometric reachability only**.
* It does not account for:

  * Self-collisions
  * Environment collisions
  * Joint torque limits
  * Dynamic constraints

It represents purely kinematic reachable space.