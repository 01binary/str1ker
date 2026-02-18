while true; do
    rostopic pub -1 arm/gripper str1ker/GripperCommand \
        '{ holdTime: 0.25 }'

    rostopic pub -1 arm/gripper str1ker/GripperCommand \
        '{ holdTime: 0.25 }'

    rostopic pub -1 arm/velocity str1ker/VelocityCommand \
        '{ base: 1, shoulder: 0, elbow: -1 }'

    rostopic pub -1 arm/gripper str1ker/GripperCommand \
        '{ holdTime: 0.25 }'

    rostopic pub -1 arm/gripper str1ker/GripperCommand \
        '{ holdTime: 0.25 }'

    rostopic pub -1 arm/velocity str1ker/VelocityCommand \
        '{ base: 0, shoulder: 0, elbow: 0 }'

    rostopic pub -1 arm/velocity str1ker/VelocityCommand \
        '{ base: -1, shoulder: 1, elbow: 1 }'

    rostopic pub -1 arm/gripper str1ker/GripperCommand \
        '{ holdTime: 0.25 }'

    rostopic pub -1 arm/gripper str1ker/GripperCommand \
        '{ holdTime: 0.25 }'

    rostopic pub -1 arm/gripper str1ker/GripperCommand \
        '{ holdTime: 0.25 }'
done