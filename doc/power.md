# Power System

## Overview

The robot can be plugged into a power outlet, run from the onboard battery, or charge the battery.

A 3-position switch selects between charging and running from the power grid when the robot is plugged in. The middle position turns off AC power.

Two circuit breakers are then used to select the power source: running from the battery or from the power supply.

![Power System Schematic](./power-schematic.svg)

## AC

The AC part of the circuit includes a voltmeter, pilot lights for power supply and battery to indicate which one is selected by the switch, and an Emergency Stop (E-Stop).

## DC

The DC part of the circuit includes the battery, the battery circuit breaker, the power supply circuit breaker, and the main robot circuit breaker.

The `24V` bus after the main breaker includes voltage and current sensing: analog front panel voltmeter/ammeter and digital meters on the main robot board that calculate and advertise the battery charge and power usage.

Most of the actuators and motors are on the `12V` bus, but the large torso servo needs at least `24V` to operate.

## Components

|Component|Description|
|-|-|
|[MEAN WELL RSP-2000-24](https://www.digikey.com/en/products/detail/mean-well-usa-inc/RSP-2000-24/7706317)|`24V`, `80A`, `1920W` Power Supply|
|[MOSEWORTH LiFePO4](https://www.amazon.com/dp/B0FX8HR52D)|`24V` `60Ah` Lithium Battery|
|[LiTime Charger](https://www.amazon.com/dp/B0BLRR73GD)|`24V` `20A` LiFePO4 Battery Charger|
|[LW28-32](https://www.amazon.com/dp/B07MZ739CS)|Changeover Switch 3 Positions 8 Terminals `690V` `32A`|
|[Eaton Bussman Circuit Breaker](https://www.eaton.com/us/en-us/catalog/emobility/series-187-marine-rated-circuit-breaker.html)|`100A` Battery and Power Supply Breakers|
|[Dihool Circuit Breaker](https://www.amazon.com/dp/B0FFFZQXWR)|`100A` Robot Breaker|
|[Daygreen A2D12C100](https://daygreen.com/products/24v-to-12v-100a-1200w-dc-dc-step-down-converter-voltage-regulator-3-year-warranty)|`24V` to `12V` `100A` `1200W` DC-DC Step Down Converter|
|[Baomain DC Analog Voltmeter](https://www.amazon.com/dp/B01KDVU1N6)|DC Bus `0-30V` Voltage Meter|
|[Baomain DC Analog Ammeter](https://www.amazon.com/dp/B01FAVYELY)|DC Bus `0-100A` Current Meter|
|[Baomain AC Analog Voltmeter](https://www.amazon.com/dp/B07S376BV3)|AC Bus `0-150V` Voltage Meter|
|[Red/Green Pilot Lights](https://www.amazon.com/dp/B0CSZ6S7LH)|AC `110V` Power Supply light, Battery Charger light|
|[E-Stop](https://www.amazon.com/dp/B079FKJG26)|AC `10A` Emergency Stop/Reset|
