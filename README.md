# ENGG1100 UFV Power & Control

This repository contains the Arduino sketch which will operate the unmanned firefighting vehicle ("UFV") prototype for an ENGG1100 project.

An Arduino Uno (CH340), a DualShock 2 wireless gamepad, and both an L298N motor driver and a proprietary PCB courtesy of UQ Innovate, will facilitate the vehicle's functions.

## What to implement
Manual control of the following components:
- ~~Drive~~
	- ~~motor direction (L2/R2)~~
- ~~Fluid Delivery~~
	- ~~pump speed (L1/R1)~~
	- ~~pump enable and disable (Square)~~
	- ~~horizontal (D-Left/Right)~~
	- ~~vertical (D-Up/Down)~~

## Excerpt from [PsxNewLib][1] regarding how to connect controller
[1]: https://github.com/SukkoPera/PsxNewLib/blob/master/README.md#tldr

> ### TL;DR
> - Power the controller at 3.3V.
> - Don't bother with 'Motor Power' pin unless you want rumble.
> - Connect all signals with proper level shifting and **1k pull-up resistors**.
> 
> ### Power
> When plugged in a real PlayStation console, controllers are powered with 3.6V on pin 5. There are (too) many tutorials and videos out there that suggest that powering the controller at 5V is just fine, but the truth is that **it is not**: PlayStation controllers are not made to work at that voltage and **they WILL break** sooner or later. You have been warned.
> 
> Now, I know 3.6V regulators aren't exactly common. 3.3V seems close enough though, there are plenty of regulators for that voltage and my experience seems to suggest that all PSX controllers work just perfectly with this slight undervoltage, which is definitely safer than powering everything at 5V. Most conveniently, almost all classic Arduinos have an onboard 3.3V regulator. It is generally rated for only "little" current (say 50 mA?), but that will be enough.
> 
> ### Data
> If you power the controller at 3.3V, all the I/O lines must also work at that voltage. Most Arduinos work at 5V, thus some level adjustment MUST be done. There are a few ways to achieve this, the easiest of which is probably using those bidirectional 4-channel level shifters you can get cheaply from China. You don't need bidirectionality, strictly speaking, still they are (almost) ideal for this application since they also provide pull-ups for the open-collector outputs and come in sets of 4.
> 
> The only issue is that these adapters generally come with 10k pull-up resistors, which are too weak and will create compatibility issues with some controllers. Therefore, **you MUST replace them with 1k ones** or put 1k in parallel to each.
> 
> Of course, if you use an Arduino board that works at 3.3V, you won't need any level shifting but **you will still need the pull-up resistors**, as the built-in ones are too weak.
> 
> A note on the *Acknowledge* pin: the original library did not use it and this is one of the reasons why it is not compatible with some controllers: it waits for a fixed interval between consecutive bytes instead of checking for the ACK pulse. Since some controllers are slower than others (typically older ones), they might not yet be ready for the next byte if the delay is not well calibrated (and it isn't).
> 
> The current version of PsxNewLib does the same, in that it does not use the ACK pin at all, but the delay was calibrated better. However this means that all controllers are polled much slower than they could be. Since this could be easily avoided by polling the ACK pin, **future versions of the library will require it** (the *devel* branch already does), so you are advised to wire it even if it is not necessary for the time being. If you don't want to waste a 4-channel module for a single signal, just connect it directly to an Arduino pin of choice and then connect a 1k resistor between it and 3.3V.

## Some useful links

[TinkerCAD simulation](https://www.tinkercad.com/things/dJLRxd4DNDG-mighty-fyyran-allis/editel?sharecode=YsRc_EckUVoIlAAD_wkochfdpelVsmDZIlHBKNxFIVg)<br>
[Arduino Uno Pin-outs](https://www.circuito.io/blog/arduino-uno-pinout/)<br>