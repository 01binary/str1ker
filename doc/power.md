# Power System

## Overview

The robot runs from the onboard battery, also charging if plugged in.

If the main breaker is off, the battery charges with the robot completely turned off.

![Power System Schematic](./power-schematic.svg)

## AC

The AC part of the circuit includes a power meter and a pilot light.

## DC

The DC part of the circuit includes the battery, breaker, contactor wired to an emergency stop, and a pilot light.

The `24V` bus after the main breaker includes voltage and current sensing: analog front panel voltmeter/ammeter and digital meters on the main robot board that calculate and advertise the battery charge and power usage.

Most of the actuators and motors are on the `12V` bus, but the large torso servo needs at least `24V` to operate.

## Components

|Component|Description|
|-|-|
|[MOSEWORTH LiFePO4](https://www.amazon.com/dp/B0FX8HR52D)|`24V` `60Ah` Lithium Battery|
|[LiTime Charger](https://www.amazon.com/dp/B0BLRR73GD)|`24V` `20A` LiFePO4 Battery Charger|
|[Dihool Circuit Breaker](https://www.amazon.com/dp/B0FFFZQXWR)|`100A` Robot Breaker|
|[Daygreen A2D12C100](https://daygreen.com/products/24v-to-12v-100a-1200w-dc-dc-step-down-converter-voltage-regulator-3-year-warranty)|`24V` to `12V` `100A` `1200W` DC-DC Step Down Converter|
|[Baomain DC Analog Voltmeter](https://www.amazon.com/dp/B01KDVU1N6)|DC Bus `0-30V` Voltage Meter|
|[Baomain DC Analog Ammeter](https://www.amazon.com/dp/B01FAVYELY)|DC Bus `0-100A` Current Meter|
|[Baomain AC Analog Voltmeter](https://www.amazon.com/dp/B07S376BV3)|AC Bus `0-150V` Voltage Meter|
|[Red/Green Pilot Lights](https://www.amazon.com/dp/B0CSZ6S7LH)|AC `110V` Power Supply light, Battery Charger light|
|[E-Stop](https://www.amazon.com/dp/B079FKJG26)|AC `600V` `10A` Emergency Stop/Reset|
