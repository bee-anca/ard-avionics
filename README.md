# ard-avionics
ARD Avionics Work 2023

All testing is done on a Teensy 4.1 over the I2C bus. 

Data rate is important in rocketry, so the goal is to minimise the `void loop()` time.

The following code is placed at the end of the `void loop()` function to track the time required (in microseconds) to complete a loop.

```c
static long last = 0;
long now = micros();
Serial.print(now - last);
last = now;
```

