# Parameter Descriptions

groups:
    -
        name: "Default"
        type: ""
        parameters:
            -
                name: "myintparam"
                type: "int"
                level: 0
                description: "My description"
                edit_method: ""
            -
                name: "myboolparam"
                type: "bool"
                level: 0
                description: "My description"
                edit_method: ""
            -
max, min, dflt:
    groups:
        -
            name: "Default"
            state: True
            id: 0
            parent: 0
    bools:
        -
            name: "myboolparam"
            value: True
    ints:
        -
            name: "myintparam"
            value: 100
