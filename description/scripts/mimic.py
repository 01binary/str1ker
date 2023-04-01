#!/usr/bin/python3

# A python script that takes four parameters from the command line: JointMin, JointMax, ActuatorMin, and ActuatorMax.
# The script then solves a system of equations with two unknowns: Multiplier and Offset.
# Multiplier is the ratio of the actuator to the joint, and Offset is the difference between the actuator and joint.
# The script then prints the values of Multiplier and Offset.
# The script is used to determine the values of Multiplier and Offset for the mimic joints in the URDF file.

import sys

# If there are less than four arguments specified on the command line, print an error and exit
if (len(sys.argv) < 4):
    print("Must be called with JointMin JointMax ActuatorMin ActuatorMax")
    exit()

# Get the four parameters from the command line
JointMin = float(sys.argv[1])
JointMax = float(sys.argv[2])
ActuatorMin = float(sys.argv[3])
ActuatorMax = float(sys.argv[4])

# Solve the system of equations
Multiplier = (JointMax - JointMin) / (ActuatorMax - ActuatorMin)
Offset = JointMin - (Multiplier * ActuatorMin)

# Print the values of Multiplier and Offset
print("Multiplier: " + str(Multiplier))
print("Offset: " + str(Offset))
