# MPU 9250 Demonstration Library.

Work credit to WOLFGANG EWALD for the base library from which we have built up off of.

Using DLPF Level 6, the teensy 4.1 loop time is 0.8168 ms at a I2C bus speed of 400 kHz with very little sensor noise.

To allow the MPU9250 to auto-calibrate while vertical on the launch rail, line 169 of `MPU6500.cpp` can be edited to reflect which axis it is "down" when the chip powers on. By default it is `z`.