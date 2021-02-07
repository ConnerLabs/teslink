# TESLINK
## PIC firmware for Tesla coil control over Toslink fibre optic
### Why bother?!
I've built plenty of DRSSTC drivers that used a wired remote control with a 9-pin cable to carry interrupter pulses, and analog control voltages for DC bus voltage and PLL fine tuning. I wanted a plug and play upgrade that would give galvanic isolation.

### How does it work?
The transmitter takes an external interrupter pulse input and also contains a DSP clone of my analog Deelux Interrupter circuit. The PIC's A/D converter is used to read 4 analog inputs: DC bus voltage and PLL fine tune are sent directly to the receiver, and PRF and burst length are used by the built-in interrupter. There is also a digital fire button input for the built-in interrupter.

All data channels are multiplexed together and Manchester encoded for transmission at approximately 1MB/s using the PIC UART. Manchester encoding is used for compatibility with AC coupled fibre optic receivers, so it should work with inexpensive Toslink receivers or pretty much any kind of fibre optic link at all.

A parity check helps to catch errors due to interference, which are all too likely around a Tesla coil. A link code is used to prevent connection of transmitters and receivers running incompatible firmware.

The receiver demultiplexes the signals and applies parity checking, link code checking and an interrupter duty cycle limit, which is hard coded and set when the PIC is programmed. The analog signals are output as PWM and the interrupter pulse output comes from the MSSP.

See comments in code for more detailed explanation.

### License
[Creative Commons Attribution-ShareAlike 4.0](https://creativecommons.org/licenses/by-sa/4.0/)
