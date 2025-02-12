# Parameter Descriptions

* rateHz                            int

* base/motor/pwmMin                 int
* base/motor/pwmMax                 int
* base/motor/stallThreshold         double
* base/motor/invert                 bool
* base/encoder/normMin              int
* base/encoder/normMax              int
* base/encoder/scaleMin             double
* base/encoder/scaleMax             double
* base/encoder/invert               bool
* base/Kp                           double
* base/Ki                           double
* base/Kd                           double
* base/iMin                         double
* base/iMax                         double
* base/tolerance                    double

* shoulder/motor/pwmMin             double
* shoulder/motor/pwmMax             double
* shoulder/motor/stallThreshold     double
* shoulder/motor/invert             bool
* shoulder/encoder/normMin          int
* shoulder/encoder/normMax          int
* shoulder/encoder/scaleMin         double
* shoulder/encoder/scaleMax         double
* shoulder/encoder/invert           bool
* shoulder/Kp                       double
* shoulder/Ki                       double
* shoulder/Kd                       double
* shoulder/iMin                     double
* shoulder/iMax                     double
* shoulder/tolerance                double

* elbow/motor/pwmMin                double
* elbow/motor/pwmMax                double
* elbow/motor/stallThreshold        double
* elbow/motor/invert                bool
* elbow/encoder/normMin             int
* elbow/encoder/normMax             int
* elbow/encoder/scaleMin            double
* elbow/encoder/scaleMax            double
* elbow/encoder/invert              bool
* elbow/Kp                          double
* elbow/Ki                          double
* elbow/Kd                          double
* elbow/iMin                        double
* elbow/iMax                        double
* elbow/tolerance                   double

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
                name: "mydoubleparam"
                type: "double"
                level: 0
                description: "My description"
                edit_method: ""
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
