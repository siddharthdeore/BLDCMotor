# BLDC Motor Dynamics and Control
Electric motor, commutation playground for three phase PMSM bldc motor

## Hardware
PCB
![Schematic](Hardware/PCBVer1.0.png)

## Motor Parameters

 Property | Variable | Value | Unit |
--- | --- | --- | ---
Viscous damping | B | 0.000052 | N-m/(rad/s) 
Rotor inertia | J | 0.0007 | Kg-m^2
Back-EMF constant | Ke | 0.0071 | V/rad/s
Torque constant | Kt | 0.0071 | N-m/A
Inductance | L | 0.005 | H
Number of Poles | P | 4 | -
Resistance | R | 3.25 | Ohm

## Field Oriented Control

### Example 1 : Angle Control

![FOC Example](https://github.com/siddharthdeore/BLDCMotor/blob/main/simulations/ex_1_foc.png)

## Space Vector Control

### Example 2 : Velocity Tracking

![FOC Example](https://github.com/siddharthdeore/BLDCMotor/blob/main/simulations/ex_2_sv.png)


# Maintainers
This repository is maintained by:

| <img src="https://avatars.githubusercontent.com/u/12745747" width="32">  | [Siddharth Deore](https://github.com/siddharthdeore) |
|--|--|

