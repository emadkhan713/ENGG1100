# ENGG1100 UFV Power & Control

This repository contains the Arduino code which will be used to operate an unmanned firefighting vehicle ("UFV") prototype, based on an Arduino Uno, DualShock 2, and L298N Motor Driver, among other components.

## Required functionality
The following components must be manually controlled:
- Fluid Delivery
	- pump speed (D-Pad Up/Down)
	- pump enable and disable (Square button)
	- horizontal bearing (Left Analog stick)
	- vertical tilt (Right Analog stick)
- Drive
	- motor speeds (fixed *or* L2/R2)
	- motor direction (L2/R2)

## Utilised libraries
PsxNewLib
DigitalIO (dependency of PsxNewLib)
L298N