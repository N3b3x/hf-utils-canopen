# HF-CANOPEN

A collection of lightweight helpers for building CANopen frames used with motor
drivers.  The code is completely self contained and includes a minimal
`CanFrame` structure so it can be integrated without external
dependencies.

## Features

- Construction of standard NMT and expedited SDO frames
- Utilities for CiA DS402 drive control objects
- Support for BLDC, stepper and DC motor controllers

These helpers can be used on bare-metal or hosted systems to compose CANopen
messages for controlling motion hardware.
