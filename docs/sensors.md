# Sensors
Sensors are at the heart of most adaptive robotics software. Without it, the robot is in the dark, and cannot employ efficient control.

## Limit Switches
The most basic form of sensor, with two states, on or off. This is essentially a glorified button thats activated when a mechanism hits its limits. These use the DIO inputs on the RoboRIO and are used to prevent a mechanism from surpassing its mechanical limits and breaking. They can also be used to zero a measured position. You initialize this with the `new DigitalInput(DIO_INPUT)` constructor.


## Relative Encoders
Another basic form of sensor, that instead reads the number of rotations of a motor shaft. On boot, this value is always zero, and increases as the motor spins counterclockwise and decreases as the motor spins clockwise. This can be used with a conversion factor to achieve the desired unit. For example, if I have an elevator, I can use the gear ratio and the pulley size to calculate the meters the elevator has traveled rather than the number of motor rotations. Most brushless motors have a built-in relative encoder that can be obtained with:

```java
motor = new CANSparkMax(MOTOR_ID);
encoder = motor.getEncoder();
```

These encoders are great but come with a few problems, mainly that they reset to zero every boot. This is great for systems where you need to figure out the speed that a motor is running at with `.getVelocity()` but not so great for accurate position readings without needing to reset the physical position of the mechanism by hand before every boot.

## Absolute Encoders
Absolute encoders are the solution to this problem. If the encoder never resets its position every boot, and retains its value persistently, the encoder is an absolute encoder. These encoders are great for pivoting mechanisms, where you need to know the exact pivot angle for repeatable control. The only problem with these sorts of encoders is that they only read values between 0-360 degrees, so you know where the shaft is rotated, but not how many times.

The solution to this is that absolute encoders aren't built into the encoder, and are usually instead mounted to the pivoting mechanism itself. [Here's an example](https://www.revrobotics.com/rev-11-1271/). These are usually plugged into the DIO ports on the RoboRIO, and you declare them with `new DutyCycleEncoder(DIO_INPUT)`
