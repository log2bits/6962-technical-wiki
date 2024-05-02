# Swerve Drive
Swerve Drive is arguably the most advanced drivetrain available for FRC, and requires proper knowledge of control theory to pull off. It's a great fusion of mechanical, electrical, and software engineering.

## What is it?
Swerve drive uses two motors in each wheel, one to spin the wheel, and one to change the wheel's direction. This allows for complicated movements that are otherwise impossible with a standard drivetrain. Strafing (moving in a direction independent of the direction of the robot) becomes possible, and rotation while moving also becomes possible. These movements are monumentally useful in almost any FRC game challenge. For example, instead of designing a complicated shooter mechanism that can pivot 360 degrees to shoot from anywhere, you can instead just use swerve drive to rotate the whole robot. This is what we did for the 2023 game challenge, Crescendo.

## How does it work?
The magic of swerve drive comes down to it's "modules". Each module resides in its corresponding corner of the robot's frame, and is where the wheel and motors are mounted. The special component is the custom gearbox that allows for the wheel to rotate along two independent axis. Thankfully, this mechanism has already been designed and is purchasable in kits, like this one: https://www.swervedrivespecialties.com/products/mk4i-swerve-module

Arguably, the easiest parts of Swerve Drive come from mechanical and electrical, as most of that is outsourced. The tricky part comes from the software. There are existing well maintained libraries out there, notably [YAGSL (Yet Another Generic Swerve Library)](https://github.com/BroncBotz3481/YAGSL) but given how closely you need to work with swerve drive to tune and accomplish complex behaviors, it is important to fully understand the software you're running. For this reason, I've decided to develop the software stack from the ground up.

## Modules first
Before anything else, make sure you've read up on Control Theory and Encoders. I'll assume you know that from here on out.

Lets start with the code for controlling a single Swerve Module.
```java
public class SwerveModule extends SubsystemBase {
  private CANSparkMax driveMotor, steerMotor;                      // NEO motors controlled by CAN SparkMaxes
  private RelativeEncoder driveEncoder, steerEncoder;              // Built-in relative encoders from NEOs
  private CANcoder absoluteSteerEncoder;                           // Absolute encoder from CANCoder on steering shaft
  private SparkPIDController drivePID, steerPID;                   // Built-in PID controllers running on-board SparkMaxes at 1000 Hz
  private SwerveModuleState targetState = new SwerveModuleState(); // Target state for the module
  private int corner;                                              // Corner of the robot the module is mounted on (0: front-left, 1: front-right, 2: back-left, 3: back-right)
  private Rotation2d absoluteSteerDirection = new Rotation2d();    // Absolute direction of the steering motor, cached and taken from the CANCoder
  private double driveVelocity = 0.0;                              // Velocity of the wheel, cached and taken from the NEO's built-in encoder
  private double drivePosition = 0.0;                              // Meters the module has driven, cached and taken from the NEO's built-in encoder
  private Rotation2d relativeSteerDirection = new Rotation2d();    // Relative direction of the steering motor, cached and taken from the NEO's built-in encoder
  private boolean isCalibrating = false;                           // Whether the module is calibrating, used to prevent driving during calibration
  
  private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward( // Feedforward Controller for the drive motor
    DRIVE_MOTOR_PROFILE.kS,
    DRIVE_MOTOR_PROFILE.kV,
    DRIVE_MOTOR_PROFILE.kA
  );

  public SwerveModule(MODULE_CONFIG config, int corner, String name) {
    this.corner = corner;

    driveMotor           = new CANSparkMax(config.CAN_DRIVE(), MotorType.kBrushless);
    steerMotor           = new CANSparkMax(config.CAN_STEER(), MotorType.kBrushless);
    absoluteSteerEncoder = new CANcoder(config.CAN_ENCODER());
    steerEncoder         = steerMotor.getEncoder();
    driveEncoder         = driveMotor.getEncoder();
    drivePID             = driveMotor.getPIDController();
    steerPID             = steerMotor.getPIDController();

    /*
     * Encoder offsets are determined by the physical mounting of the module on the robot.
     * This means that modules can be swapped between corners without needing to recalibrate the encoders.
     */
    double encoderOffset = config.ENCODER_OFFSET();
    switch (corner) {
      case 0:
        encoderOffset += 0.0;
        break;
      case 1:
        encoderOffset += 0.25;
        break;
      case 2:
        encoderOffset += -0.25;
        break;
      case 3:
        encoderOffset += 0.5;
        break;
      default:
    }
    encoderOffset %= 2; // Normalize the offset to be between 0 and 2
    encoderOffset = (encoderOffset > 1.0) ? encoderOffset - 2.0 : (encoderOffset < -1.0) ? encoderOffset + 2.0 : encoderOffset; // Normalize the offset to be between -1 and 1
    
    MagnetSensorConfigs magConfig = new MagnetSensorConfigs(); // CANCoder is configured with MagnetSensorConfigs object all at once
    magConfig.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf); // Absolute sensor range is set to -180 to 180 degrees, instead of 0 to 360
    magConfig.withMagnetOffset(encoderOffset); // Magnet offset is set to the calculated encoder offset
    BaseStatusSignal.setUpdateFrequencyForAll(50, absoluteSteerEncoder.getAbsolutePosition(), absoluteSteerEncoder.getFaultField(), absoluteSteerEncoder.getVersion()); // CANCoder is set to update at 50 Hz, and only give data we care about
    absoluteSteerEncoder.optimizeBusUtilization(); // Apply bus frequency settings

    /*
     * Using custom logging tool, SparkMaxUtil, to configure and log the SparkMaxes and CANCoders.
     * The drive motor is set to have a calculated current limit that prevents the wheel from slipping by preventing the motor
     * from applying more torque than the friction of the wheel.
     * The steer motor is set to coast mode, as it doesn't need to hold position when disabled. This also prevents tearing up the carpet.
     */
    SparkMaxUtil.configureAndLog(this, driveMotor, false, CANSparkMax.IdleMode.kBrake, PHYSICS.SLIPLESS_CURRENT_LIMIT, PHYSICS.SLIPLESS_CURRENT_LIMIT);
    SparkMaxUtil.configureAndLog(this, steerMotor, true, CANSparkMax.IdleMode.kCoast);
    SparkMaxUtil.configureEncoder(driveMotor, SWERVE_DRIVE.DRIVE_ENCODER_CONVERSION_FACTOR); // Configure the drive encoder to be in meters rather than rotations
    SparkMaxUtil.configureEncoder(steerMotor, SWERVE_DRIVE.STEER_ENCODER_CONVERSION_FACTOR); // Configure the steer encoder to be in radians rather than rotations
    SparkMaxUtil.configurePID(this, driveMotor, DRIVE_MOTOR_PROFILE.kP, DRIVE_MOTOR_PROFILE.kI, DRIVE_MOTOR_PROFILE.kD, 0.0, false);
    SparkMaxUtil.configurePID(this, steerMotor, STEER_MOTOR_PROFILE.kP, STEER_MOTOR_PROFILE.kI, STEER_MOTOR_PROFILE.kD, 0.0, true); // Steer motor is continuous (-180 and 180 degrees wrap), so it needs to be configured as such
    
    // Save config and burn to flash
    SparkMaxUtil.save(driveMotor);
    SparkMaxUtil.save(steerMotor);
    
    // Reduce CAN bus utilization by only sending necessary data
    SparkMaxUtil.configureCANStatusFrames(driveMotor, true, true);
    SparkMaxUtil.configureCANStatusFrames(steerMotor, false, true);

    // Set the steer encoder in the motor to the absolute position of the CANCoder
    seedSteerEncoder();

    // Log the module's data
    String logPath = "module" + name + "/";
    Logger.autoLog(this, logPath + "relativeSteerDirection",           () -> relativeSteerDirection.getDegrees());
    Logger.autoLog(this, logPath + "absoluteSteerDirection",        () -> absoluteSteerDirection.getDegrees());

    // Add status checks for the module
    StatusChecks.addCheck(this, name + "canCoderHasFaults", () -> absoluteSteerEncoder.getFaultField().getValue() == 0);
    StatusChecks.addCheck(this, name + "canCoderIsConnected", () -> absoluteSteerEncoder.getVersion().getValue() != 0);
  }


  public void periodic() {
    /*
     * Update the module's state variables with the latest data from the SparkMaxes and CANCoders.
     * This is important because calling get() on the SparkMaxes and CANCoders is expensive, so we only want to do it once per loop.
     */
    relativeSteerDirection = Rotation2d.fromRadians(steerEncoder.getPosition());
    absoluteSteerDirection = Rotation2d.fromRotations(absoluteSteerEncoder.getAbsolutePosition().getValue());
    driveVelocity = driveEncoder.getVelocity();
    drivePosition = driveEncoder.getPosition();

    // Only drive the module if the system is enabled and the robot is not calibrating
    if (!ENABLED_SYSTEMS.ENABLE_DRIVE) return;
    if (isCalibrating) return;
    drive(targetState);

    // Stop motors if the battery voltage is too low
    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.SWERVE_DRIVE) stop();
  }
  
  public void drive(SwerveModuleState state) {
    double speedMetersPerSecond = state.speedMetersPerSecond;
    double radians = state.angle.getRadians();
    
    // Slow down the drive motor when the steering angle is far from the target angle.
    if (SWERVE_DRIVE.DO_ANGLE_ERROR_SPEED_REDUCTION) {
      speedMetersPerSecond *= Math.cos(SwerveMath.angleDistance(radians, getMeasuredState().angle.getRadians()));
    }
    
    // Using on-board SparkMax PID controllers to control the drive and steer motors. This allows for 1kHz closed loop control.
    drivePID.setReference(
      speedMetersPerSecond,
      CANSparkMax.ControlType.kVelocity,
      0,
      driveFF.calculate(speedMetersPerSecond) // Add our own feedforward instead of using the SparkMax's built-in feedforward. This is because the built-in feedforward does not support the kS term.
    );
    steerPID.setReference(
      radians,
      CANSparkMax.ControlType.kPosition
    );

    // If the module is not moving, seed the steer encoder with the absolute position of the steer CANCoder
    if (state.speedMetersPerSecond == 0 && Math.abs(getRelativeSteerDirection().minus(getAbsoluteSteerDirection()).getDegrees()) > 0.5) {
      seedSteerEncoder();
    }
  }
  
  /**
   * Sets the target state for the module to drive at.
   * @param state target state
   */
  public void setTargetState(SwerveModuleState state) {
    // Optimize the state to flip the steering angle if it is faster to go the other way. This ensures that the module always takes the shortest path to the target angle.
    targetState = SwerveModuleState.optimize(state, getMeasuredState().angle);
  }
  
  /**
   * Stops the module by setting the target state to 0 speed and the current angle.
   */
  public void stop() {
    targetState = new SwerveModuleState(0.0, getMeasuredState().angle);
  }
  
  /**
   * Seeds the position of the built-in relative encoder with the absolute position of the steer CANCoder.
   * This is because the CANCoder polls at a lower rate than we'd like, so we essentially turn the relative encoder into an fast-updating absolute encoder.
   * Also the built-in SparkMaxPIDControllers require a compatible encoder to run the faster 1kHz closed loop 
   */
  public void seedSteerEncoder() {
    steerEncoder.setPosition(getAbsoluteSteerDirection().getRadians());
  }

  /**
   * Gets the direction of the steering motor from the built-in relative encoder.
   * @return relative direction of the steering motor
   */
  public Rotation2d getRelativeSteerDirection() {
    return relativeSteerDirection;
  }
  
  /**
   * Gets the absolute direction of the steering motor from the CANCoder.
   * @return absolute direction of the steering motor
   */
  private Rotation2d getAbsoluteSteerDirection() {
    return absoluteSteerDirection;
  }

  /**
   * Gets the target state for the module to drive at.
   * @return target state
   */
  public SwerveModuleState getTargetState() {
    return targetState;
  }
  
  /**
   * Gets the measured state of the module, which is the current velocity and angle of the module.
   * @return measured state
   */
  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(driveVelocity, getAbsoluteSteerDirection());
  }

  /**
   * Gets the position of the module, which is the distance the wheel has traveled and the angle of the module.
   * @return module position
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(drivePosition, getMeasuredState().angle);
  }

  /**
   * Calculates the velocity of the wheel from the power applied to the motor.
   * @param power power applied to the motor (-1.0, 1.0)
   * @return velocity of the wheel in meters per second
   */
  public static double calcWheelVelocity(double power) {
    return power * Constants.SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY;
  }
  
  /**
   * Gets the field-relative pose of the module.
   * @param robotPose pose of the robot on the field
   * @return field-relative pose of the module
   */
  public Pose2d getPose(Pose2d robotPose) {
    Pose2d relativePose = new Pose2d();
    if (corner == 0) relativePose = new Pose2d(
      SWERVE_DRIVE.WHEELBASE / 2.0,
      SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    if (corner == 1) relativePose = new Pose2d(
      SWERVE_DRIVE.WHEELBASE / 2.0,
      -SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    if (corner == 2) relativePose = new Pose2d(
      -SWERVE_DRIVE.WHEELBASE / 2.0,
      SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    if (corner == 3) relativePose = new Pose2d(
      -SWERVE_DRIVE.WHEELBASE / 2.0,
      -SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    return relativePose.relativeTo(new Pose2d(
      new Translation2d(),
      robotPose.getRotation().times(-1.0)
    )).relativeTo( new Pose2d(
      -robotPose.getX(),
      -robotPose.getY(),
      new Rotation2d()
    ));
  }
}
```

### Basics first
I know it looks like a lot, but I've added comments to almost everything so hopefully things at least make a little bit more sense. The basic principle is that there are two motors, and two PID Controllers. Each PID Controller is configured separately, and tuned for the purpose of either steering or driving. When driving, the `setTargetState` method is called, telling the swerve module to attempt to reach that state with the PID Controllers. Aside from that, I also added a few more advanced modifications:

### CAN Bus Utilization Optimizations
The CAN bus is the way almost all data on the robot is sent, and with swerve drive, there are 12 different devices all sending their own data. This alone could potentially saturate the CAN bus with so much data that it won't be able to handle many other subsystems, and start dropping packets. This is very bad. To prevent this, I've employed a few tactics to reduce the bus utilization. First though, its important you understand how data is sent over the CAN bus.

Every device on the bus has an update rate, which is to say a rate that their data is sent. Instead of a request and receive type of interaction where the code asks for the readings on an encoder, the encoder's data is sent periodically regardless if you actually need it or not. This is great for making sure that there is little delay in the code for getting data, but is sometimes awful for the bandwidth on the CAN bus. Often times the devices are sending much more data than is actually needed, and this is a problem. To address this, you can change the update rate of different parts of the data being sent. That's what is happening here:

```java
MagnetSensorConfigs magConfig = new MagnetSensorConfigs(); // CANCoder is configured with MagnetSensorConfigs object all at once
magConfig.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf); // Absolute sensor range is set to -180 to 180 degrees, instead of 0 to 360
magConfig.withMagnetOffset(encoderOffset); // Magnet offset is set to the calculated encoder offset
BaseStatusSignal.setUpdateFrequencyForAll(50, absoluteSteerEncoder.getAbsolutePosition(), absoluteSteerEncoder.getFaultField(), absoluteSteerEncoder.getVersion()); // CANCoder is set to update at 50 Hz, and only give data we care about
absoluteSteerEncoder.optimizeBusUtilization(); // Apply bus frequency settings

// Reduce CAN bus utilization by only sending necessary data
SparkMaxUtil.configureCANStatusFrames(driveMotor, true, true);
SparkMaxUtil.configureCANStatusFrames(steerMotor, false, true);
```

Here, I've made sure the CANCoder only sends data for `getAbsolutePosition`, `getFaultField`, and `getVersion`, and only 50 times a second. I've also made sure the steer encoder sends data on it's position and ignores data on the velocity, temperature, voltage, and current.

Finally, the last change I've made regards caching values. Each time you fetch data on a sensor or any data over the CAN bus, it takes a bit of time for it to get the value. I'm not entirely sure why this is, as it's supposed to be cached, but this caused a few headaches when we ran into loop overruns in the past. If in `getMeasuredState`, we were to just call `steerEncoder.getPosition()`, it would result in dozens of calls to that position every loop, as the measured state is requested many times. To prevent this, we just make sure to only call it once per loop, and save it to a variable here to be reused later:
```java
/*
* Update the module's state variables with the latest data from the SparkMaxes and CANCoders.
* This is important because calling get() on the SparkMaxes and CANCoders is expensive, so we only want to do it once per loop.
*/
relativeSteerDirection = Rotation2d.fromRadians(steerEncoder.getPosition());
absoluteSteerDirection = Rotation2d.fromRotations(absoluteSteerEncoder.getAbsolutePosition().getValue());
driveVelocity = driveEncoder.getVelocity();
drivePosition = driveEncoder.getPosition();
```

### Motor Current Limiting
Another thing to keep in mind with so many motors running at once, is the strain on the battery. The battery can't expel an infinite amount of energy at once, and the more current drawn from the battery, the lower the voltage dips. This is called voltage sag. Eventually, if the system draws too many amps at once, a brownout occurs. This is when the voltage drops below 6.8V, and gets to the point where the RoboRIO cannot function properly anymore. This causes all systems on the robot to shut off, and can lead to many nasty problems.

To combat this, we limit the amount of current each of our motors can draw. By default the motors draw a max of 80 A each, so a total of 650A at 12V, which is 7,800 W of power, much more than the battery can handle. Actually, 80 A is far too much for the motor to handle as well, and if under load, can cause the motor to fail and destroy itself. You can read more about that here: https://www.revrobotics.com/neo-brushless-motor-locked-rotor-testing/

There are actually two types of current limits, "free", and "stall". The free current limit is the limit applied while the motor is spinning, while the stall current limit is applied while the motor is attempting to spin, but is stationary. Usually the latter occurs under a heavy load or when hitting a mechanical limit. This type of current limit is the scariest, and has the highest chance of burning out the motor. This is because the motor is expending a lot of energy to try to move, but without any movement, the energy is released as heat, slowly killing the motor. Usually a good rule of thumb is to have a 40 A stall current limit, and 60 A free current limit for NEOs

Our situation is unique though, as we can use the current limit to our advantage. Even with an infinitely powerful motor, our robot can only accelerate as fast as the friction between the carpet and the wheels will allow, which is a calculable limit. But first, we must understand what the current and voltage actually mean.

The current applied to a motor dictates it's torque. With more current, the motor is able to move heavier loads. The voltage on the other hand, dictates the speed of the motor. At 12V, the motor will spin as fast as it can, but with voltage sag, the motor will be unable to reach its top speed. Knowing the mass of the robot, the friction on the ground, and a few key specs of the motor, we can calculate the current limit that limits the motor's torque to drive at max speed without breaking free of the carpet and slipping.
```java
public static final double SLIPLESS_ACCELERATION = 9.80 * FRICTION_COEFFICIENT;
public static final int SLIPLESS_CURRENT_LIMIT = (int) ((SLIPLESS_ACCELERATION * NEO.STATS.stallCurrentAmps * ROBOT_MASS * WHEEL_RADIUS) / (4.0 * DRIVE_MOTOR_GEARING * NEO.STATS.stallTorqueNewtonMeters));
```
This is extremely convenient because instead of experimentally determining the current limit, and retesting any time the robot's weight changes, we have a formula that works and dynamically adjusts to the robot's weight and the surface that it's on.

### Global Encoder Offsets
The way the absolute encoders (CANCoders) work is they read the polarity of a magnet in the drive shaft of whatever is rotating. In this case, its the steering axis, and there are small cylindrical magnets in each module. These magnets rotate as the wheel changes direction, allowing the encoder to know which direction the wheel is pointed in at all times. Importantly though, the magnets initial configuration is random, as they are just placed in the motor shaft irrespective of the wheels position. To compensate for this, we use encoder offsets.

Encoder offsets are measured values for how much to shift the readings of the encoder by. For our case, we point the wheel in the direction of zero degrees, measure what the encoder says and take the negative of that as our offset. This way whenever we read values from the CANCoder, it'll be accurate to the position of the wheel rather than the random orientation of the magnet.

This works in theory, but in practice it can be annoying because each module itself is rotated differently depending on which corner of the chassis its mounted to. In our code, instead of dealing with robot-relative encoder offsets, we have specific module-relative offsets and add to them depending on which corner the module is located in. This is what that looks like:

```java
/*
 * Encoder offsets are determined by the physical mounting of the module on the robot.
 * This means that modules can be swapped between corners without needing to recalibrate the encoders.
 */
double encoderOffset = config.ENCODER_OFFSET();
switch (corner) {
  case 0:
    encoderOffset += 0.0;
    break;
  case 1:
    encoderOffset += 0.25;
    break;
  case 2:
    encoderOffset += -0.25;
    break;
  case 3:
    encoderOffset += 0.5;
    break;
  default:
}
encoderOffset %= 2; // Normalize the offset to be between 0 and 2
encoderOffset = (encoderOffset > 1.0) ? encoderOffset - 2.0 : (encoderOffset < -1.0) ? encoderOffset + 2.0 : encoderOffset; // Normalize the offset to be between -1 and 1
```

### Separate Feedforward Controller
The `SparkMaxPIDController` has a built-in feedforward controller, but only supports the kF term, and doesn't support kS. It works alright without it, but slow movements are hard without the kS term because the motor power is too low to get the wheels to start moving. For this reason, I've set the kF term for the built-in PID controller to 0.0 and add a WPILib feedforward back in later:

```java
// Using on-board SparkMax PID controllers to control the drive and steer motors. This allows for 1kHz closed loop control.
drivePID.setReference(
    speedMetersPerSecond,
    CANSparkMax.ControlType.kVelocity,
    0,
    driveFF.calculate(speedMetersPerSecond) // Add our own feedforward instead of using the SparkMax's built-in feedforward. This is because the built-in feedforward does not support the kS term.
);
```

### Angle Error Speed Reduction
One potential problem is that it can take a while for the PID controller for steering to finally reach it's setpoint, and if the controller for driving achieves it's velocity setpoint before the steering controller does, the whole drivetrain can travel in a totally different direction than intended. This can be mitigated by reducing the speed of the drive motor based on how far the steering controller is away from it's setpoint:

```java
// Slow down the drive motor when the steering angle is far from the target angle.
if (SWERVE_DRIVE.DO_ANGLE_ERROR_SPEED_REDUCTION) {
    speedMetersPerSecond *= Math.cos(SwerveMath.angleDistance(radians, getMeasuredState().angle.getRadians()));
}
```

### Steer Encoder Seeding


### 