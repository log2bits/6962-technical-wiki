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
// Calculates the power (-1.0 - 1.0) sent to the motor 
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
// Calculates the power (-1.0 - 1.0) sent to the motor 
// given the current measurement and target setpoint
public double calculate(double measurement, double setpoint, double tolerance) {
  if (setpoint < measurement - tolerance) return -1.0;
  if (setpoint > measurement + tolerance) return 1.0;
  return 0.0;
}
```

Congratulations! We've just made a **Bang-Bang Controller**! This is the most simple of all control system algorithms and has many flaws.

## A new idea.

The primary problem with Bang-Bang Controllers is that they have no sense of motor speed, and maximize the output power in any situation. This is a problem because you get very jerky and unpredictable movement, and we often overshoot our target state, requiring a lower tolerance than we'd like.

A simple solution to this would be to slow down the motor speed as we get closer to our setpoint, something like this:

```java
// Calculates the power (-1.0 - 1.0) sent to the motor 
// given the current measurement and target setpoint
public double calculate(double measurement, double setpoint) {
  double error = measurement - setpoint;
  return error;
}
```

This takes the error (measurement - setpoint) and uses that as the motor power. If the error is small, the motor will run slower. If the error is negative, the motor will run backward.

This works in some cases, but depending on the unit of measurement, we get vastly different results. For example, if the measurement and setpoint are measured in motor rotations, the motor will be given full power only a single rotation away from the setpoint, essentially turning this into a Bang-Bang Controller again. The opposite is true as well. Given a really large unit, you end up with the motor ramping down too early, resulting in a less-than-optimal response time.

## Proportional Term

To compensate for different types of systems with different units and different magnitudes of speeds and inertia, we introduce a **Proportional Term**. By *term*, I just mean an additional parameter that decides how this control system behaves.

This term acts as a scaling factor for the error, simply multiplying the error by the term results in the output motor power, like so:

```java
// Calculates the power (-1.0 - 1.0) sent to the motor 
// given the current measurement and target setpoint
public double calculate(double measurement, double setpoint, double kP) {
  double error = measurement - setpoint;
  return error * kP;
}
```

This acts as a tuning parameter to adjust the system's responsiveness. If the error is large, the result of the multiplication by Kp is also large, resulting in a large adjustment and faster approach towards the setpoint. If the error is small, the response is less intense, allowing the system to approach the setpoint more gently with less chance of overshooting.

However, a P controller alone can still fall short in two main scenarios:

1. **Steady-state error**: In some circumstances, a proportional controller will still have a residual error at steady state, meaning that the system doesn't perfectly reach the target. This is due to the fact that as the error decreases, the control action decreases proportionally and may not be enough to overcome system disturbances or friction.

2. **Overshoot**: While the proportional controller reduces the chances of overshooting compared to the previous controllers, it does not entirely eliminate it. Large values of Kp, while responsive, increase the likelihood of overshooting the setpoint.

## Derivative Term

To compensate for overshoot, we can introduce a new term, the derivative term. This term uses the rate of change (slope) of the error to compensate for the predicted future error. The math behind this term gets a little complicated, so 