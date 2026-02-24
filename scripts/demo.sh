echo "Press ENTER to stop"

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

if read -t 0.1 -n 1 input < /dev/tty; then
    if [ "$input" = "" ]; then
        echo "Stopping."
        break
    fi
fi

rostopic pub -1 arm/velocity str1ker/VelocityCommand \
    '{ base: 0, shoulder: 0, elbow: 0 }'