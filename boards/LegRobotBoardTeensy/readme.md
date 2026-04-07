# Leg Robot Board

A board that controls the mobile platform with four mecanum wheels:

+ Leg Actuators
+ Leg Actuator Potentiometers
+ Leg Wheels
+ Leg Wheel Encoders
+ 9-DOF IMU

## Modules

The following components are placed onto the board as modules:

|Module|Function|
|-|-|
|[Teensy 4.0](https://www.sparkfun.com/teensy-4-0.html)|[ROS](https://www.ros.org/) Node|
|[Adafruit 9-DOF IMU](https://www.adafruit.com/product/2472)|[TF2](https://wiki.ros.org/tf2) Transform|

## Devices

The following external components are connected to the board via `JST-XH` locking connectors:

|Device|Function|
|-|-|
|8x [IBT2 Motor Drivers](https://www.amazon.com/2-Piece-High-Power-Limiting-Function-Suitable/dp/B0DXKXKYRK)|Wheel Motor Drivers, Leg Actuator Drivers (4-pin connector: `EN`, `RPWM`, `LPWM`, `VT`)|
|4x [Feedback Rod Linear Actuator](https://www.firgelliauto.com/products/feedback-rod-actuator?variant=2632742851) Potentiometers|Leg Actuator Potentiometers (3-pin connector: `3V3`, `SIG`, `GND`)|
|4x [Leg (Wheel) Encoder Board](../LegEncoderBoard/readme.md)|Wheel Encoders (4-pin x 2-row IDC connector: `A`, `B`, `I`, `3V3`, and four `GND`'s|

## Functionality

### PWM

There are four legs and each has two motors: a linear lift actuator and a brushed DC motor that runs the wheel.

[PCA9685PW,118](https://www.digikey.com/en/products/detail/nxp-usa-inc/pca9685pw-118/2034325) PWM driver is used to run this large number of motors over `I2C` bus to avoid over-allocating Teensy pins.

### ADC

The linear lift actuators have built-in multi-turn potentiometers. These are routed to Teensy `ADC` pins.

### Quadrature

The [wheel encoders](../LegEncoderBoard/readme.md) are connected to each of the four wheels, outputting `A`, `B`, and `I` pulses counting rotations (`1000` pulses per revolution).

## Interface

## Pins

| Pin            | Function                       | Group                     |
| -------------- | ------------------------------ | ------------------------- |
| `A0`           | Actuator 1 Potentiometer `SIG` | Front Left Leg (Actuator) |
| `D0`           | Quadrature Encoder 1 `A`       | Front Left Leg (Wheel)    |
| `D1`           | Quadrature Encoder 1 `B`       | Front Left Leg (Wheel)    |
| `D22`          | Quadrature Encoder 1 `Index`   | Front Left Leg (Wheel)    |
| `D6`           | Motor Driver 1 `EN`            | Front Left Leg (Wheel)    |
| `D9`           | Motor Driver 2 `EN`            | Front Left Leg (Actuator) |
| `PCA9685 CH0`  | Motor Driver 1 `RPWM`          | Front Left Leg (Wheel)    |
| `PCA9685 CH1`  | Motor Driver 1 `LPWM`          | Front Left Leg (Wheel)    |
| `PCA9685 CH2`  | Motor Driver 2 `RPWM`          | Front Left Leg (Actuator) |
| `PCA9685 CH3`  | Motor Driver 2 `LPWM`          | Front Left Leg (Actuator) |
| `A1`           | Actuator 2 Potentiometer `SIG` | Front Right Leg (Actuator)|
| `D2`           | Quadrature Encoder 2 `A`       | Front Right Leg (Wheel)   |
| `D3`           | Quadrature Encoder 2 `B`       | Front Right Leg (Wheel)   |
| `D23`          | Quadrature Encoder 2 `Index`   | Front Right Leg (Wheel)   |
| `D10`          | Motor Driver 3 `EN`            | Front Right Leg (Wheel)   |
| `D11`          | Motor Driver 4 `EN`            | Front Right Leg (Actuator)|
| `PCA9685 CH4`  | Motor Driver 3 `RPWM`          | Front Right Leg (Wheel)   |
| `PCA9685 CH5`  | Motor Driver 3 `LPWM`          | Front Right Leg (Wheel)   |
| `PCA9685 CH6`  | Motor Driver 4 `RPWM`          | Front Right Leg (Actuator)|
| `PCA9685 CH7`  | Motor Driver 4 `LPWM`          | Front Right Leg (Actuator)|
| `A2`           | Actuator 3 Potentiometer `SIG` | Rear Left Leg (Actuator)  |
| `D4`           | Quadrature Encoder 3 `A`       | Rear Left Leg (Wheel)     |
| `D5`           | Quadrature Encoder 3 `B`       | Rear Left Leg (Wheel)     |
| `D24`          | Quadrature Encoder 3 `Index`   | Rear Left Leg (Wheel)     |
| `D12`          | Motor Driver 5 `EN`            | Rear Left Leg (Wheel)     |
| `D13`          | Motor Driver 6 `EN`            | Rear Left Leg (Actuator)  |
| `PCA9685 CH8`  | Motor Driver 5 `RPWM`          | Rear Left Leg (Wheel)     |
| `PCA9685 CH9`  | Motor Driver 5 `LPWM`          | Rear Left Leg (Wheel)     |
| `PCA9685 CH10` | Motor Driver 6 `RPWM`          | Rear Left Leg (Actuator)  |
| `PCA9685 CH11` | Motor Driver 6 `LPWM`          | Rear Left Leg (Actuator)  |
| `A3`           | Actuator 4 Potentiometer `SIG` | Rear Right Leg (Actuator) |
| `D7`           | Quadrature Encoder 4 `A`       | Rear Right Leg (Wheel)    |
| `D8`           | Quadrature Encoder 4 `B`       | Rear Right Leg (Wheel)    |
| `D25`          | Quadrature Encoder 4 `Index`   | Rear Right Leg (Wheel)    |
| `D20`          | Motor Driver 7 `EN`            | Rear Right Leg (Wheel)    |
| `D21`          | Motor Driver 8 `EN`            | Rear Right Leg (Actuator) |
| `PCA9685 CH12` | Motor Driver 7 `RPWM`          | Rear Right Leg (Wheel)    |
| `PCA9685 CH13` | Motor Driver 7 `LPWM`          | Rear Right Leg (Wheel)    |
| `PCA9685 CH14` | Motor Driver 8 `RPWM`          | Rear Right Leg (Actuator) |
| `PCA9685 CH15` | Motor Driver 8 `LPWM`          | Rear Right Leg (Actuator) |
| `A5`           | I2C `SCL` (PCA9685)            | PWM Servo Driver          |
| `A4`           | I2C `SDA` (PCA9685)            | PWM Servo Driver          |
| `D26`          | BNO055 `RST`                   | Accelerometer             |

## Bill of Materials

|Component|Description|
|-|-|
|[PCA9685PW,118](https://www.digikey.com/en/products/detail/nxp-usa-inc/pca9685pw-118/2034325)|PWM Multiplexer|
|[150080BS75000](https://www.digikey.com/en/products/detail/w%C3%BCrth-elektronik/)|Blue LED for `LPWM`/`RPWM` signals|
|[NCD0603R1](https://www.lcsc.com/product-detail/C84263.html?s_z=s_C84263)|Red LED for `EN` signals|
|[RCG06031K00FKEA](https://www.digikey.com/en/products/detail/vishay-dale/rcg06031k00fkea/4172389)|LED Resistor 1K|
|[RC0603FR-07150RL](https://www.digikey.com/en/products/detail/yageo/RC0603FR-07150RL/726958)|LED Resistor 150 Ohm|
|[RC0603FR-07470RL](https://www.digikey.com/en/products/detail/yageo/RC0603FR-07470RL/727256)|LED Resistor 470 Ohm|
|[CRCW060310K0FKEA](https://www.digikey.com/en/products/detail/vishay-dale/crcw060310k0fkea/1174782)|Pullup Resistor 10K|
|[CRCW0603100KFKEA](https://www.digikey.com/en/products/detail/vishay-dale/crcw0603100kfkea/1174896)|Series Resistor 100K|
|[SN74HC08DR](https://www.digikey.com/en/products/detail/texas-instruments/sn74hc08dr/276834)|IC Gate|
|[MMBT3904LT1G](https://www.digikey.com/en/products/detail/onsemi/MMBT3904LT1G/919601)|LED Transistor|
|[TLC555CDR](https://www.digikey.com/en/products/detail/texas-instruments/tlc555cdr/276979)|555 Timer|
|[RC0603FR-07360KL](https://www.digikey.com/en/products/detail/yageo/rc0603fr-07360kl/727183)|555 Timer Resistor 360K|
|[CC0603JRX7R7BB105](https://www.digikey.com/en/products/detail/yageo/CC0603JRX7R7BB105/7164369)|555 Timer Capacitor 1uF|
|[C0402C103J4RACTU](https://www.digikey.com/en/products/detail/kemet/C0402C103J4RACTU/411041)|555 Timer Capacitor 0.01uF|
|[JST_XH_B3B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b3b-xh-a/1651046)|Knee Potentiometers (`3V3`, `SIG`, `GND`)|
|[JST_XH_B4B-XH-A](https://www.digikey.com/en/products/detail/jst-sales-america-inc/b4b-xh-a/1651047)|Leg Actuator Drivers, Leg Wheel Drivers (`LPWM`, `RPWM`, `L_EN`, `R_EN`)|
