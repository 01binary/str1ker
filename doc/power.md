# Power System

## Overview

The robot runs from the battery, charging if plugged in. A circuit breaker or an E-Stop can be used to turn off the robot while the battery is charging.

![Power System Schematic](./power-schematic.svg)

## AC

The AC part of the circuit includes an energy meter and a green pilot light.

## DC

The DC part of the circuit includes the battery, breaker, contactor wired to an emergency stop, and a red pilot light.

The `24V` bus after the main breaker includes voltage and current sensing: analog front panel voltmeter/ammeter and digital meters on the main robot board that calculate and advertise the battery charge and power usage.

Most of the actuators and motors are on the `12V` bus, but the large torso servo needs at least `24V` to operate.

## Components

|Component|Description|
|-|-|
|[Moseworth LiFePO4](https://www.amazon.com/dp/B0FX8HR52D)|`24V` `60Ah` Lithium Battery|
|[LiTime Charger](https://www.amazon.com/dp/B0BLRR73GD)|`24V` `20A` LiFePO4 Battery Charger|
|[D52-2066](https://www.amazon.com/dp/B094F98PYF)|AC Bus `110V` `100A` Energy Meter|
|[Baomain AC Analog Voltmeter](https://www.amazon.com/dp/B07S376BV3)|AC Bus `0-150V` Voltage Meter|
|[Green AC Pilot Light](https://www.amazon.com/dp/B0CSZ6S7LH)|AC Bus `110V` Supply Light|
|[E-Stop](https://www.amazon.com/dp/B079FKJG26)|DC Bus Emergency Stop/Reset|
|[TXBD1-100](https://www.amazon.com/dp/B0C53PMC7D)|DC Bus Circuit Breaker `2P` `100A`|
|[Contactor](https://www.amazon.com/dp/B0F6BZVVFX)|DC Bus `1000V` `150A` Contactor|
|[Inline Fuse](https://www.amazon.com/dp/B0C15H3F9M)|DC `5A` Inline Fuse for Contactor Coil|
|[Daygreen A2D12C100](https://daygreen.com/products/24v-to-12v-100a-1200w-dc-dc-step-down-converter-voltage-regulator-3-year-warranty)|`24V` to `12V` `100A` `1200W` DC-DC Step Down Converter|
|[Baomain DC Analog Voltmeter](https://www.amazon.com/dp/B01KDVU1N6)|DC Bus `0-30V` Voltage Meter|
|[Baomain DC Analog Ammeter](https://www.amazon.com/dp/B01FAVYELY)|DC Bus `0-100A` Current Meter|
|[Red DC Pilot Light](https://www.amazon.com/dp/B0CC4R2VR7)|DC Bus `24V` Supply Light|
