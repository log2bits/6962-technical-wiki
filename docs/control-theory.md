# Control Theory

Control theory is a major part of robotics and harnesses powerful techniques that allow for precise and controlled mechanical actuation. If you have a mechanism, and you wish to have it reach a desired state, you need control theory in some way or another.

Pivoting arm? Rotating turret? Spinning flywheel? Linear elevator? All of these mechanisms require knowledge of control theory to function correctly.

## So what is it?

Simply put, control theory is a way of reading the current state of a mechanism with a sensor and then deciding how to run a motor to achieve the target state.

A Control system usually involves the following components:

1. **Reference:** Desired behavior or final condition you want the system to achieve. Otherwise known as the target state or the setpoint.

2. **Plant:** The system that you want to control.

3. **Controller:** The device or algorithm that decides the system's output based on the reference.

4. **Sensor:** Measures the current state of the plant.

5. **Feedback:** Comparison of the plant's current state to the reference, which then alters the system to help it get closer to the reference.

## Let's try an example.

Assume we have an elevator and a sensor that magically tells us how many meters along the track the elevator is currently sitting. Currently, it's reading 2 meters, but we want to get to 10 meters. This 10-meter target is the **Reference**. Okay, so what do you do? You move up, right? So you tell the motor to run at full speed, and eventually, it reaches 10 meters and stops. This is what that logic would look like:

```java
// Calculates the power (-1.0, 1.0) sent to the motor 
// given the current measurement and target setpoint
public double calculate(double measurement, double setpoint) {
  if (setpoint < measurement) return -1.0;
  if (setpoint > measurement) return 1.0;
  return 0.0;
}
```

In an ideal world, this would work great. But we assume a few things that aren't true in the real world:

1. Sensor readings aren't instant and perfectly precise

2. Motor commands aren't instant

3. Mechanisms have inertia

4. Our software can only run so many times a second

Because of this oversight, our mechanism will likely overshoot its target, attempt to compensate, and undershoot, repeating this cycle forever. This is called an **oscillation**. The system will **oscillate** around its target state but never fully stop at it.

So what can we do?

One simple fix is to add some tolerance. Our logic only allows the motor to stop if the measurement reads EXACTLY what our target state is. With this fix, the motor will stop when its "close enough", reducing the likelihood that it skips over the setpoint.

```java
// Calculates the power (-1.0, 1.0) sent to the motor 
// given the current measurement and target setpoint
public double calculate(double measurement, double setpoint, double tolerance) {
  if (setpoint < measurement - tolerance) return -1.0;
  if (setpoint > measurement + tolerance) return 1.0;
  return 0.0;
}
```

Congratulations! We've just made a Bang Bang Controller! This is the most simple of all control system algorithms and has many flaws.