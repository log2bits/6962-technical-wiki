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
There are two encoders used for steering, the absolute CANCoder, and the relative built-in NEO encoder. Both of these work, but only the relative encoder is used for the onboard SparkMAX PID loop. This is a problem, because we have the absolute encoder for a reason, and if we cant use it with the PID controller, it's effectively useless.

A solution to this is to "seed" the relative encoder. This is done by overriding the relative encoder's position with the data readout from the absolute encoder. This effectively converts the relative encoder into an absolute encoder! This is what that looks like:

```java
/**
 * Seeds the position of the built-in relative encoder with the absolute position of the steer CANCoder.
 * This is because the CANCoder polls at a lower rate than we'd like, so we essentially turn the relative encoder into an fast-updating absolute encoder.
 * Also the built-in SparkMaxPIDControllers require a compatible encoder to run the faster 1kHz closed loop 
 */
public void seedSteerEncoder() {
  steerEncoder.setPosition(getAbsoluteSteerDirection().getRadians());
}
```

We call this whenever the module is stationary:

```java
// If the module is not moving, seed the steer encoder with the absolute position of the steer CANCoder
if (state.speedMetersPerSecond == 0 && Math.abs(getRelativeSteerDirection().minus(getAbsoluteSteerDirection()).getDegrees()) > 0.5) {
  seedSteerEncoder();
}
```

## Swerve Drive Class
Moving on, we need a SwerveDrive class that handles all four SwerveModule objects. Here's what that looks like:

```java
public class SwerveDrive extends SubsystemBase {

  // Swerve kinematics
  public SwerveModule[] modules = new SwerveModule[SWERVE_DRIVE.MODULE_COUNT];
  private SwerveDriveKinematics kinematics = getKinematics();
  private CustomSwerveDrivePoseEstimator poseEstimator;
  private static Field2d field = new Field2d(); // Field2d object for SmartDashboard widget
  private ChassisSpeeds drivenChassisSpeeds = new ChassisSpeeds();
  private SWERVE_DRIVE.MODULE_CONFIG[] equippedModules;
  private SwerveDriveWheelPositions previousWheelPositions;
  private Translation2d linearAcceleration;

  // Gyro & rotation
  private static AHRS gyro; // NAVX2-MXP Gyroscope
  private Rotation2d gyroHeading = Rotation2d.fromDegrees(0.0);
  private Rotation2d gyroOffset = SWERVE_DRIVE.STARTING_POSE.get().getRotation();
  private Debouncer doneRotating = new Debouncer(0.5);
  private double addedAlignmentAngularVelocity = 0.0;
  private double angularAcceleration = 0.0;
  private PIDController alignmentController = new PIDController(
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kP,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kI,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kD
  );

  // Autonomous
  private Supplier<Translation2d> rotationOverridePoint = null;
  private Rotation2d rotationOverrideOffset = new Rotation2d();

  // States
  private boolean isAligning = false;
  private boolean parked = false;
  private boolean parkingDisabled = false;
  private boolean isDriven = false;
  private boolean gyroConnected = false;
  
  public SwerveDrive() {
    // Create the serve module objects
    equippedModules = SWERVE_DRIVE.IS_PROTOTYPE_CHASSIS ? SWERVE_DRIVE.EQUIPPED_MODULES_PROTOTYPE : SWERVE_DRIVE.EQUIPPED_MODULES_COMPETITION;
    int corner = 0;
    for (SWERVE_DRIVE.MODULE_CONFIG config : equippedModules) {
      String name = SWERVE_DRIVE.MODULE_NAMES[corner];
      if (RobotBase.isSimulation()) { // Use sim modules in simulation
        modules[corner] = new SwerveModuleSim(config, corner, name);
      } else {
        modules[corner] = new SwerveModule(config, corner, name);
      }
      corner++;
    }

    // Set up pose estimator and rotation controller
    poseEstimator = new CustomSwerveDrivePoseEstimator(
      kinematics,
      SWERVE_DRIVE.STARTING_POSE.get().getRotation(),
      getModulePositions().positions,
      SWERVE_DRIVE.STARTING_POSE.get(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2)),
      VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(30))
    );

    previousWheelPositions = getModulePositions(); // Used for twist calculations for odometry
    alignmentController.enableContinuousInput(-Math.PI, Math.PI); // Set the controller to be continuous
    alignmentController.setTolerance(SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.TOLERANCE.getRadians());
    
    // If possible, connect to the gyroscope
    try {
      gyro = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), false);
    }

    // Gyro takes a little while to calibrate, so we wait a second before setting the offset
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        gyroOffset = gyroOffset.minus(gyro.getRotation2d());
      } catch (Exception e) {}
    }).start();
    
    
    // Log data
    SmartDashboard.putData("Field", field);
    Logger.autoLog(this, "pose", () -> this.getPose());
    Logger.autoLog(this, "measuredHeading", () -> this.getHeading().getDegrees());
    Logger.autoLog(this, "targetHeading", () -> Units.radiansToDegrees(alignmentController.getSetpoint()));
    Logger.autoLog(this, "targetStates", () -> getTargetModuleStates());
    Logger.autoLog(this, "measuredStates", () -> getMeasuredModuleStates());
    Logger.autoLog(this, "modulePositions", () -> getModulePositions());
    Logger.autoLog(this, "gyroAcceleration", () -> Math.hypot(gyro.getWorldLinearAccelX(), gyro.getWorldLinearAccelY()));
    Logger.autoLog(this, "gyroVelocity", () -> Math.hypot(gyro.getVelocityX(), gyro.getVelocityY()));
    Logger.autoLog(this, "commandedLinearAcceleration", () -> linearAcceleration.getNorm());
    Logger.autoLog(this, "commandedLinearVelocity", () -> Math.hypot(getDrivenChassisSpeeds().vxMetersPerSecond, getDrivenChassisSpeeds().vyMetersPerSecond));
    Logger.autoLog(this, "commandedAngularAcceleration", () -> angularAcceleration);
    Logger.autoLog(this, "commandedAngularVelocity", () -> getDrivenChassisSpeeds().omegaRadiansPerSecond);
    Logger.autoLog(this, "measuredAngularVelocity", () -> getMeasuredChassisSpeeds().omegaRadiansPerSecond);
    Logger.autoLog(this, "measuredLinearVelocity", () -> Math.hypot(getMeasuredChassisSpeeds().vxMetersPerSecond, getMeasuredChassisSpeeds().vyMetersPerSecond));
    Logger.autoLog(this, "gyroIsCalibrating", () -> gyro.isCalibrating());
    Logger.autoLog(this, "gyroIsConnected", () -> gyro.isConnected());
    Logger.autoLog(this, "gyroRawDegrees", () -> gyro.getRotation2d().getDegrees());
    StatusChecks.addCheck(this, "isGyroConnected", gyro::isConnected);

    // Path planner setup
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getMeasuredChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kP, SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kI, SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kD), // Translation PID constants
        new PIDConstants(SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kP, SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kI, SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kD), // Rotation PID constants
        SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY, // Max module speed, in m/s
        SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
      ),
      () -> false,
      this // Reference to this subsystem to set requirements
    );

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        field.getObject("Target Pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
        // Do whatever you want with the poses here
        field.getObject("Active Path").setPoses(poses);
    });
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_DRIVE) return;
    // If the robot is disabled, reset states and target heading
    if (RobotState.isDisabled()) {
      for (SwerveModule module : modules) {
        module.seedSteerEncoder();
      }
      setTargetHeading(getHeading());
      isAligning = false;
      rotationOverridePoint = null;
    }
    
    updateOdometry();

    // Update field
    FieldObject2d modulesObject = field.getObject("Swerve Modules");
    Pose2d[] modulePoses = new Pose2d[SWERVE_DRIVE.MODULE_COUNT];
    Pose2d robotPose = getPose();
    int i = 0;
    for (SwerveModule module : modules) {
      modulePoses[i] = module.getPose(robotPose);
      i++;
    }
    modulesObject.setPoses(modulePoses);
    field.setRobotPose(getPose());
    
    // If drive calls are not being made, stop the robot
    if (!isDriven) driveFieldRelative(0.0, 0.0, 0.0);
    isDriven = false;
  }

  public void updateOdometry() {
    Pose2d poseBefore = getPose();

    // Math to use the swerve modules to calculate the robot's rotation if the gyro disconnects
    SwerveDriveWheelPositions wheelPositions = getModulePositions();
    Twist2d twist = kinematics.toTwist2d(previousWheelPositions, wheelPositions);
    Pose2d newPose = getPose().exp(twist);
    if (!gyroConnected && (gyro.isConnected() && !gyro.isCalibrating())) {
      gyroOffset = gyroHeading.minus(gyro.getRotation2d());
    }
    gyroConnected = gyro.isConnected() && !gyro.isCalibrating();
    if (gyroConnected && !RobotBase.isSimulation()) {
      gyroHeading = gyro.getRotation2d();
    } else {
      gyroHeading = gyroHeading.plus(newPose.getRotation().minus(getPose().getRotation()));
    }

    poseEstimator.update(gyroHeading.plus(gyroOffset), getModulePositions());
    AprilTags.injectVisionData(LIMELIGHT.APRILTAG_CAMERA_POSES, this); // Limelight vision data from apriltags

    // Sometimes the vision data will cause the robot to go crazy, so we check if the robot is moving too fast and reset the pose if it is
    Pose2d currentPose = getPose();
    double magnitude = currentPose.getTranslation().getNorm();
    if (magnitude > 1000 || Double.isNaN(magnitude) || Double.isInfinite(magnitude)) {
      System.out.println("BAD");
      LEDs.setState(LEDs.State.BAD);
      resetPose(gyroHeading.plus(gyroOffset), poseBefore, previousWheelPositions);
    }
    
    previousWheelPositions = wheelPositions.copy();
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Drives the robot at a given field-relative velocity
   * @param xVelocity [meters / second] Positive x is away from your alliance wall
   * @param yVelocity [meters / second] Positive y is to your left when standing behind the alliance wall
   * @param angularVelocity [radians / second] Rotational velocity, positive spins counterclockwise
   */
  public void driveFieldRelative(double xVelocity, double yVelocity, double angularVelocity) {
    driveFieldRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
  }

  /**
   * 
   * Drives the robot at a given field-relative ChassisSpeeds
   * @param fieldRelativeSpeeds
   */
  private void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveAttainableSpeeds(fieldRelativeSpeeds);
  }

  /**
   * Drives the robot at a given robot-relative velocity
   * @param xVelocity [meters / second] Positive x is towards the robot's front
   * @param yVelocity [meters / second] Positive y is towards the robot's left
   * @param angularVelocity [radians / second] Rotational velocity, positive spins counterclockwise
   */
  public void driveRobotRelative(double xVelocity, double yVelocity, double angularVelocity) {
    driveRobotRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
  }

  /**
   * Drives the robot at a given robot-relative ChassisSpeeds
   * @param robotRelativeSpeeds
   */
  private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    driveFieldRelative(ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getAllianceAwareHeading()));
  }

  /**
   * Drives the robot at a given field-relative ChassisSpeeds. This is the base method, and all other drive methods call this one.
   * @param fieldRelativeSpeeds The desired field-relative speeds
   */
  private void driveAttainableSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
    isDriven = true;

    // Avoid obstacles
    if (!(RobotState.isAutonomous() && !Autonomous.avoidPillars)) {
      Translation2d velocity = XBoxSwerve.avoidObstacles(new Translation2d(
        fieldRelativeSpeeds.vxMetersPerSecond,
        fieldRelativeSpeeds.vyMetersPerSecond
      ), this);
      fieldRelativeSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), fieldRelativeSpeeds.omegaRadiansPerSecond);
    }

    // If the robot is rotating, cancel the rotation override
    if (fieldRelativeSpeeds.omegaRadiansPerSecond > 0 && !RobotState.isAutonomous()) {
      rotationOverridePoint = null;
    }

    // If the robot is in autonomous mode, or if a rotation override point is set, stop the robot from rotating
    if (rotationOverridePoint != null || RobotState.isAutonomous()) {
      fieldRelativeSpeeds.omegaRadiansPerSecond = 0.0;
      if (rotationOverridePoint != null) facePoint(rotationOverridePoint.get(), rotationOverrideOffset);
    }
    
    // Conditionals to compensate for the slop in rotation when aligning with PID controllers
    if (Math.abs(fieldRelativeSpeeds.omegaRadiansPerSecond) > 0.01) {
      setTargetHeading(getHeading());
      isAligning = false;
    }
    if (!isAligning && doneRotating.calculate(Math.abs(getDrivenChassisSpeeds().omegaRadiansPerSecond) < 0.1)) {
      setTargetHeading(getHeading());
      isAligning = true;
    }
    
    // Calculate the angular velocity to align with the target heading
    double alignmentAngularVelocity = alignmentController.calculate(getHeading().getRadians()) + addedAlignmentAngularVelocity;
    addedAlignmentAngularVelocity = 0.0;
    if (isAligning && !alignmentController.atSetpoint() && !parked) fieldRelativeSpeeds.omegaRadiansPerSecond += alignmentAngularVelocity;

    // Calculate the wheel speeds to achieve the desired field-relative speeds
    SwerveDriveWheelStates wheelSpeeds = kinematics.toWheelSpeeds(fieldRelativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      wheelSpeeds.states,
      fieldRelativeSpeeds,
      SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY,
      SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY,
      SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_VELOCITY
    );
    fieldRelativeSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

    // Limit translational acceleration
    Translation2d targetLinearVelocity = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    Translation2d currentLinearVelocity = new Translation2d(drivenChassisSpeeds.vxMetersPerSecond, drivenChassisSpeeds.vyMetersPerSecond);
    linearAcceleration = (targetLinearVelocity).minus(currentLinearVelocity).div(Robot.getLoopTime());
    double linearForce = linearAcceleration.getNorm() * SWERVE_DRIVE.ROBOT_MASS;

    // Limit rotational acceleration
    double targetAngularVelocity = fieldRelativeSpeeds.omegaRadiansPerSecond;
    double currentAngularVelocity = drivenChassisSpeeds.omegaRadiansPerSecond;
    angularAcceleration = (targetAngularVelocity - currentAngularVelocity) / Robot.getLoopTime();
    double angularForce = Math.abs((SWERVE_DRIVE.PHYSICS.ROTATIONAL_INERTIA * angularAcceleration) / SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS);
    
    // Limit the total force applied to the robot
    double frictionForce = 9.80 * SWERVE_DRIVE.ROBOT_MASS * SWERVE_DRIVE.FRICTION_COEFFICIENT;
    if (linearForce + angularForce > frictionForce) {
      double factor = (linearForce + angularForce) / frictionForce;
      linearAcceleration = linearAcceleration.div(factor);
      angularAcceleration /= factor;
    }

    // Calculate the attainable linear and angular velocities and update the driven chassis speeds
    Translation2d attainableLinearVelocity = currentLinearVelocity.plus(linearAcceleration.times(Robot.getLoopTime()));
    double attainableAngularVelocity = currentAngularVelocity + (angularAcceleration * Robot.getLoopTime());
    drivenChassisSpeeds = new ChassisSpeeds(attainableLinearVelocity.getX(), attainableLinearVelocity.getY(), attainableAngularVelocity);
    
    // Discretize converts a continous-time chassis speed into discrete-time to compensate for the loop time. This helps reduce drift when rotating while driving in a straight line.
    drivenChassisSpeeds = ChassisSpeeds.discretize(drivenChassisSpeeds, Robot.getLoopTime());
    SwerveDriveWheelStates drivenModuleStates = kinematics.toWheelSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(drivenChassisSpeeds, getAllianceAwareHeading()));
    
    // If the robot is not moving, park the modules
    boolean moving = false;
    for (SwerveModuleState moduleState : kinematics.toWheelSpeeds(fieldRelativeSpeeds).states) if (Math.abs(moduleState.speedMetersPerSecond) > 0.0) moving = true;
    for (SwerveModuleState moduleState : drivenModuleStates.states) if (Math.abs(moduleState.speedMetersPerSecond) > 0.0) moving = true;
    parked = false;

    if (!moving) {
      if (!parkingDisabled) {
        parkModules();
        return;
      }
      // If the robot is aligning, park the modules in a different configuration.
      // Parking normally while aligning can cause the robot to drift away from the setpoint.
      if (alignmentController.atSetpoint()) {
        parkForAlignment();
      }
    }

    parkingDisabled = false;
    driveModules(drivenModuleStates);
  }
  
  /**
   * Drives the swerve modules at the calculated speeds
   * @param wheelSpeeds The calculated speeds and directions for each module
   */
  private void driveModules(SwerveDriveWheelStates wheelSpeeds) {
    // Drive the swerve modules at the calculated speeds
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      modules[i].setTargetState(wheelSpeeds.states[i]);
    }
  }

  /**
   * Sets the target heading for the robot
   * @param heading The target heading for the robot
   */
  public void setTargetHeading(Rotation2d heading) {
    alignmentController.setSetpoint(heading.getRadians());
    parkingDisabled = true;
  }

  /**
   * Sets the target heading for the robot
   * @param heading The target heading for the robot
   */
  public void setTargetHeadingAndVelocity(Rotation2d heading, double velocity) {
    setTargetHeading(heading);
    addedAlignmentAngularVelocity = velocity;
  }

  /**
   * Faces a point on the field
   * @param point The point on the field we want to face
   */
  public Command facePointCommand(Supplier<Translation2d> point, Rotation2d rotationOffset) {
    return Commands.run(
      () -> facePoint(point.get(), rotationOffset)
    );
  }

  /**
   * Faces a point on the field
   * @param point
   * @param rotationOffset
   */
  public void facePoint(Translation2d point, Rotation2d rotationOffset) {
    double time = 0.02;

    // With no point, do nothing.
    if (point == null) {
      setTargetHeadingAndVelocity(getHeading(), 0.0);
      return;
    }

    // If the robot is close to the point, do nothing.
    if (point.getDistance(getPose().getTranslation()) < 1.0 && RobotState.isAutonomous()) {
      return;
    }

    // Calculate the future position of the robot, and predict how the heading will need to change in the future.
    Translation2d currentPosition = getPose().getTranslation();
    Translation2d futurePosition = getPose().getTranslation().plus(getFieldVelocity().times(time));
    Rotation2d currentTargetHeading = point.minus(currentPosition).getAngle().plus(rotationOffset);
    Rotation2d futureTargetHeading = point.minus(futurePosition).getAngle().plus(rotationOffset);    
    double addedVelocity = futureTargetHeading.minus(currentTargetHeading).getRadians() / time;
    if (getPose().getTranslation().getDistance(point) < 1.0) {
      addedVelocity = 0.0;
    }
    setTargetHeadingAndVelocity(currentTargetHeading, addedVelocity);
  }


  /**
   * 
   * @return The target heading for the robot
   */
  public Rotation2d getTargetHeading() {
    return Rotation2d.fromRadians(alignmentController.getSetpoint());
  }

  /**
   * Sets a rotation point override so the robot always points in that direction. Used for autonomous mostly.
   * @param point The point to face
   * @param rotationOffset The offset to add to the rotation. 180 degrees would point away from it.
   */
  public void setRotationTargetOverrideFromPoint(Supplier<Translation2d> point, Rotation2d rotationOffset) {
    rotationOverridePoint = point;
    rotationOverrideOffset = rotationOffset;
    addedAlignmentAngularVelocity = 0.0;
  }

  /**
   * This creates an "X" pattern with the wheels which makes the robot very hard to move
   */
  private void parkModules() {
    modules[0].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    modules[1].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    modules[2].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    modules[3].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    parked = true;
  }

  /**
   * This creates an "O" pattern with the wheels which makes the robot very hard to translate, but still allows for rotation
   */
  private void parkForAlignment() {
    modules[0].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    modules[1].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    modules[2].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    modules[3].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
  }

  /**
   * Resets the odometer position to a given position
   * @param pose Position to reset the odometer to
   */
  public void resetPose(Rotation2d heading, Pose2d pose, SwerveDriveWheelPositions wheelPositions) {
    poseEstimator.resetPosition(heading, wheelPositions, pose);
    alignmentController.setSetpoint(getHeading().getRadians());
  }

  /**
   * Resets the odometer position to a given position
   * @param pose Position to reset the odometer to
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    alignmentController.setSetpoint(getHeading().getRadians());
  }

  /**
   * Checks if the robot can zero its heading
   * @return
   */
  public boolean canZeroHeading() {
    return (parked || isAligning || RobotState.isDisabled()) && (Math.abs(getRotationalVelocity()) < 0.5);
  }

  /**
   * 
   * @param visionMeasurement The robot position on the field from the apriltags
   */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp, Matrix<N3,N1> visionMeasurementStdDevs) {
    Rotation2d oldHeading = getHeading();
    poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    poseEstimator.addVisionMeasurement(visionMeasurement, timestamp);
    Rotation2d newHeading = getHeading();
    alignmentController.setSetpoint(Rotation2d.fromRadians(alignmentController.getSetpoint()).plus(newHeading).minus(oldHeading).getRadians());
  }

  /**
   * Stops all motors on all modules
   */
  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  /**
   * Returns the field-relative velocity of the robot
   * @return Field-relative velocity in m/s
   */
  public Translation2d getFieldVelocity() {
    ChassisSpeeds fieldRelativeChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getMeasuredChassisSpeeds(), getHeading());
    return new Translation2d(fieldRelativeChassisSpeeds.vxMetersPerSecond, fieldRelativeChassisSpeeds.vyMetersPerSecond);
  }

  /**
   * Returns the rotational velocity of the robot
   * @return Rotational velocity in rad/s
   */
  public double getRotationalVelocity() {
    return getMeasuredChassisSpeeds().omegaRadiansPerSecond;
  }

  /**
   * @return Target chassis x, y, and rotational velocity (robot-relative)
   */
  private ChassisSpeeds getTargetChassisSpeeds() {
    return kinematics.toChassisSpeeds(getTargetModuleStates());
  }

  /**
   * @return Measured chassis x velocity, y velocity, and rotational velocity (robot-relative)
   */
  private ChassisSpeeds getMeasuredChassisSpeeds() {
    return kinematics.toChassisSpeeds(getMeasuredModuleStates());
  }

  /**
   * @return Driven chassis x speed, y speed, and rotational speed (robot-relative)
   */
  private ChassisSpeeds getDrivenChassisSpeeds() {
    return drivenChassisSpeeds;
  }

  /**
   * @return Measured module positions
   */
  public SwerveDriveWheelPositions getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      positions[i] = modules[i].getModulePosition();
    }
    return new SwerveDriveWheelPositions(positions);
  }

  /**
   * @return Target module states (speed and direction)
   */
  private SwerveDriveWheelStates getTargetModuleStates() {
    SwerveModuleState[] targetStates = new SwerveModuleState[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      targetStates[i] = modules[i].getTargetState();
    }
    return new SwerveDriveWheelStates(targetStates);
  }

  /**
   * @return Measured module states (speed and direction)
   */
  private SwerveDriveWheelStates getMeasuredModuleStates() {
    SwerveModuleState[] measuredStates = new SwerveModuleState[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      measuredStates[i] = modules[i].getMeasuredState();
    }
    return new SwerveDriveWheelStates(measuredStates);
  }

  /**
   * @return This swerve drive's NavX AHRS IMU Gyro
   */
  public static AHRS getGyro() {
    return gyro;
  }

  /**
   * Resets gyro heading
   */
  public void resetGyroHeading(Rotation2d newHeading) {
    gyroOffset = newHeading.minus(gyroHeading);
    alignmentController.reset();
    alignmentController.setSetpoint(newHeading.getRadians());
  }

  /**
   * @return Gyro heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * @return Heading as a Rotation2d based on alliance
   */
  public Rotation2d getAllianceAwareHeading() {
    return getHeading().plus(Rotation2d.fromDegrees(Constants.IS_BLUE_TEAM.get() ? 0.0 : 180.0));
  }

  /**
   * @return Pose on the field from odometer data as a Pose2d
   */
  public Pose2d getPose() {
    Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
    return estimatedPose;
  }

  /**
   * Get the pose of the robot at a given timestamp in the past (up to 1 or 2 seconds ago)
   * @param timestampSeconds The timestamp to get the pose at (Timer.getFPGATimestamp() is the current timestamp)
   * @return Pose on the field from odometer data as a Pose2d
   */
  public Pose2d getPose(double timestampSeconds) {
    Pose2d estimatedPose = poseEstimator.getEstimatedPosition(timestampSeconds);
    return estimatedPose;
  }

  /**
   * Predicts the future pose of the robot based on the current velocity. This pose is if the robot suddenly decelerates to 0 m/s.
   * @return Future pose on the field
   */
  public Pose2d getFuturePose() {
    Translation2d futurePosition = getPose().getTranslation();
    futurePosition = futurePosition.plus(getFieldVelocity().times(getFieldVelocity().getNorm()).div(2.0 * Constants.SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION));
    return new Pose2d(futurePosition, getPose().getRotation());
  }

  /**
   * Patch solution to be more accurate under the stage
   * @return
   */
  public Pose2d getFuturePoseStage() {
    Translation2d futurePosition = getPose().getTranslation();
    futurePosition = futurePosition.plus(getFieldVelocity().times(getFieldVelocity().getNorm()).div(1.0 * Constants.SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION));
    return new Pose2d(futurePosition, getPose().getRotation());
  }

  /**
   * Is the robot under the stage?
   * @return
   */
  public boolean underStage() {
    if (!RobotState.isAutonomous()) {
      return MathUtils.isInsideTriangle(Field.BLUE_STAGE_CORNERS[0], Field.BLUE_STAGE_CORNERS[1], Field.BLUE_STAGE_CORNERS[2], getFuturePose().getTranslation()) ||
             MathUtils.isInsideTriangle(Field.RED_STAGE_CORNERS[0], Field.RED_STAGE_CORNERS[1], Field.RED_STAGE_CORNERS[2], getFuturePose().getTranslation()) ||
             MathUtils.isInsideTriangle(Field.BLUE_STAGE_CORNERS[0], Field.BLUE_STAGE_CORNERS[1], Field.BLUE_STAGE_CORNERS[2], getPose().getTranslation()) ||
             MathUtils.isInsideTriangle(Field.RED_STAGE_CORNERS[0], Field.RED_STAGE_CORNERS[1], Field.RED_STAGE_CORNERS[2], getPose().getTranslation());
    } else {
      return MathUtils.isInsideTriangle(Field.BLUE_STAGE_CORNERS[0], Field.BLUE_STAGE_CORNERS[1], Field.BLUE_STAGE_CORNERS[2], getFuturePoseStage().getTranslation()) ||
             MathUtils.isInsideTriangle(Field.RED_STAGE_CORNERS[0], Field.RED_STAGE_CORNERS[1], Field.RED_STAGE_CORNERS[2], getFuturePoseStage().getTranslation());
    }
  }

  /**
   * @return Field2d object for SmartDashboard widget.
   */
  public static Field2d getField() {
    return field;
  }

  /**
   * Converts the speed of a wheel moving to the angular velocity of the robot as if it's
   * rotating in place
   * @param wheelSpeed Drive velocity in m/s
   * @return Equivalent rotational velocity in rad/s
   * @see #toLinear(double)
   */
  public static double toAngular(double wheelSpeed) {
    return wheelSpeed / SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS;
  }

  /**
   * Converts the angular velocity of the robot to the speed of a wheel moving as if the
   * robot is rotating in place
   * @param angularVelocity Rotational velocity in rad/s
   * @return Equivalent drive velocity in m/s
   * @see #toAngular(double)
   */
  public static double toLinear(double angularVelocity) {
    return angularVelocity * SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS;
  }

  /**
   * Create a kinematics object for the swerve drive based on the SWERVE_DRIVE constants
   * @return A SwerveDriveKinematics object that models the swerve drive
  */
  public static SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(
      new Translation2d( SWERVE_DRIVE.TRACKWIDTH / 2.0, SWERVE_DRIVE.WHEELBASE  / 2.0), 
      new Translation2d( SWERVE_DRIVE.TRACKWIDTH / 2.0, -SWERVE_DRIVE.WHEELBASE / 2.0), 
      new Translation2d(-SWERVE_DRIVE.TRACKWIDTH / 2.0, SWERVE_DRIVE.WHEELBASE  / 2.0), 
      new Translation2d(-SWERVE_DRIVE.TRACKWIDTH / 2.0, -SWERVE_DRIVE.WHEELBASE / 2.0));
  }

  /**
   * Pathfind to a first given point on the field, and then follow a straight line to the second point.
   * @param firstPoint
   * @param secondPoint
   * @return
   */
  public Command pathfindThenFollowPath(Pose2d firstPoint, Pose2d secondPoint) {
    Rotation2d angle = secondPoint.getTranslation().minus(firstPoint.getTranslation()).getAngle();

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(firstPoint.getTranslation(), angle),
      new Pose2d(secondPoint.getTranslation(), angle)
    );

    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
      new GoalEndState(
        0.0,
        secondPoint.getRotation(),
        true
      )
    );
    
    return AutoBuilder.pathfindThenFollowPath(
      path,
      SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }

  /**
   * Go to a position on the field (without object avoidence)
   * @param pose Field-relative pose on the field to go to
   * @param xboxController Xbox controller to cancel the command
   * @return A command to run
   */
  public Command goToSimple(Pose2d pose) {
    Rotation2d angle = pose.getTranslation().minus(getPose().getTranslation()).getAngle();

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(getPose().getTranslation(), angle),
      new Pose2d(pose.getTranslation(), angle)
    );

    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
      new GoalEndState(
        0.0,
        pose.getRotation(),
        true
      )
    );

    return Commands.sequence(
      AutoBuilder.followPath(path),
      runOnce(() -> setTargetHeading(pose.getRotation()))
    );
  }
}
```

### Kinematics
A lot of the programming that goes into swerve has to do with the math behind how the module states are decided.
This is done with kinematics, and thankfully, WPILib has great build-in support for it.

The kinematics is defined like this:
```java
private SwerveDriveKinematics kinematics = getKinematics();
```
where `getKinematics` is:
```java
/**
 * Create a kinematics object for the swerve drive based on the SWERVE_DRIVE constants
 * @return A SwerveDriveKinematics object that models the swerve drive
 */
public static SwerveDriveKinematics getKinematics() {
  return new SwerveDriveKinematics(
    new Translation2d( SWERVE_DRIVE.TRACKWIDTH / 2.0, SWERVE_DRIVE.WHEELBASE  / 2.0), 
    new Translation2d( SWERVE_DRIVE.TRACKWIDTH / 2.0, -SWERVE_DRIVE.WHEELBASE / 2.0), 
    new Translation2d(-SWERVE_DRIVE.TRACKWIDTH / 2.0, SWERVE_DRIVE.WHEELBASE  / 2.0), 
    new Translation2d(-SWERVE_DRIVE.TRACKWIDTH / 2.0, -SWERVE_DRIVE.WHEELBASE / 2.0));
}
```
The `SwerveDriveKinematics` object is part of WPILib, and simply takes in a set of four positions for the swerve modules relative to the center of the robot.
The order of these positions is very important, and it must be Front Left, Front Right, Back Left, Back Right.
This kinematics object is then used to convert a desired chassis movement to the individual states sent to the swerve modules:
```java
SwerveDriveWheelStates moduleStates = kinematics.toWheelSpeeds(robotRelativeChassisSpeeds);
```

### ChassisSpeeds
`ChassisSpeeds` is the common way to store a given movement of the swerve drive. It contains a translational velocity and rotational velocity. Importantly, this can either be relative to the robot (positive is always forward), or relative to the field (positive is always away from the alliance wall). This can get tricky, so be sure which frame of reference the ChassisSpeeds object is when you're programming.

### Odometry
An odometer is a device that is commonly found in cars to measure how many miles they've driven. The same goes for swerve drive. Because we can measure how far each wheel travels, and we have a gyroscope, we can theoretically calculate exactly where the robot is on the field. In practice, this method is prone to drift, and often becomes more inaccurate as the match goes on, especially if the robot is collided with. These collisions, and even just quick acceleration and stopping can cause the wheels to slip, giving the odometer false data.

We can mitigate this with a Kalman Filter. A kalman filter is a mathematical model to fuse multiple sources of data. If we use this with vision data from the cameras, we can get a pretty reasonable estimate of our position on the field. This is called a `PoseEstimator`.

This is how that is initialized:
```java
// Set up pose estimator and rotation controller
poseEstimator = new CustomSwerveDrivePoseEstimator(
  kinematics,
  SWERVE_DRIVE.STARTING_POSE.get().getRotation(), // Starting rotation
  getModulePositions().positions, // The accumulated distance so far on each wheel
  SWERVE_DRIVE.STARTING_POSE.get(), // Starting pose
  VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2)), // Margin of error for odometry
  VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(30)) // Margin of error for vision pose estimates (apriltags)
);
```

And then, we periodically feed it data:
```java
poseEstimator.update(gyroHeading.plus(gyroOffset), getModulePositions());
```

And when we want to know the position on the field, we call:
```java
/**
 * @return Pose on the field from odometer data as a Pose2d
 */
public Pose2d getPose() {
  Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
  return estimatedPose;
}
```

Here's all the odometry code, including some that I've added in to compensate for gyro disconnects and bad vision data:

```java
public void updateOdometry() {
  Pose2d poseBefore = getPose();

  // Math to use the swerve modules to calculate the robot's rotation if the gyro disconnects
  SwerveDriveWheelPositions wheelPositions = getModulePositions();
  Twist2d twist = kinematics.toTwist2d(previousWheelPositions, wheelPositions);
  Pose2d newPose = getPose().exp(twist);
  if (!gyroConnected && (gyro.isConnected() && !gyro.isCalibrating())) {
    gyroOffset = gyroHeading.minus(gyro.getRotation2d());
  }
  gyroConnected = gyro.isConnected() && !gyro.isCalibrating();
  if (gyroConnected && !RobotBase.isSimulation()) {
    gyroHeading = gyro.getRotation2d();
  } else {
    gyroHeading = gyroHeading.plus(newPose.getRotation().minus(getPose().getRotation()));
  }

  poseEstimator.update(gyroHeading.plus(gyroOffset), getModulePositions());
  AprilTags.injectVisionData(LIMELIGHT.APRILTAG_CAMERA_POSES, this); // Limelight vision data from apriltags

  // Sometimes the vision data will cause the robot to go crazy, so we check if the robot is moving too fast and reset the pose if it is
  Pose2d currentPose = getPose();
  double magnitude = currentPose.getTranslation().getNorm();
  if (magnitude > 1000 || Double.isNaN(magnitude) || Double.isInfinite(magnitude)) {
    System.out.println("BAD");
    LEDs.setState(LEDs.State.BAD);
    resetPose(gyroHeading.plus(gyroOffset), poseBefore, previousWheelPositions);
  }
  
  previousWheelPositions = wheelPositions.copy();
}
```

### Drive Attainable Speeds
There are two ways to drive SwerveDrive, field-relative, and robot-relative. Most of the time, we drive the robot using field-relative controls, but we still need robot-relative occasionally.
Commanding the drivetrain to drive is done using one of the four public methods:
```java
/**
 * Drives the robot at a given field-relative velocity
 * @param xVelocity [meters / second] Positive x is away from your alliance wall
 * @param yVelocity [meters / second] Positive y is to your left when standing behind the alliance wall
 * @param angularVelocity [radians / second] Rotational velocity, positive spins counterclockwise
 */
public void driveFieldRelative(double xVelocity, double yVelocity, double angularVelocity) {
  driveFieldRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
}

/**
 * 
 * Drives the robot at a given field-relative ChassisSpeeds
 * @param fieldRelativeSpeeds
 */
private void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
  driveAttainableSpeeds(fieldRelativeSpeeds);
}

/**
 * Drives the robot at a given robot-relative velocity
 * @param xVelocity [meters / second] Positive x is towards the robot's front
 * @param yVelocity [meters / second] Positive y is towards the robot's left
 * @param angularVelocity [radians / second] Rotational velocity, positive spins counterclockwise
 */
public void driveRobotRelative(double xVelocity, double yVelocity, double angularVelocity) {
  driveRobotRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
}

/**
 * Drives the robot at a given robot-relative ChassisSpeeds
 * @param robotRelativeSpeeds
 */
private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
  driveFieldRelative(ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getAllianceAwareHeading()));
}
```

Now you might notice, that these methods all eventually call each other, and then call `driveAttainableSpeeds`. This method post-processes the drive commands to allow for smoother driving and desired movement that is actually attainable given the physics at play. Here's that method:

```java
/**
 * Drives the robot at a given field-relative ChassisSpeeds. This is the base method, and all other drive methods call this one.
 * @param fieldRelativeSpeeds The desired field-relative speeds
 */
private void driveAttainableSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
  isDriven = true;

  // Avoid obstacles
  if (!(RobotState.isAutonomous() && !Autonomous.avoidPillars)) {
    Translation2d velocity = XBoxSwerve.avoidObstacles(new Translation2d(
      fieldRelativeSpeeds.vxMetersPerSecond,
      fieldRelativeSpeeds.vyMetersPerSecond
    ), this);
    fieldRelativeSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), fieldRelativeSpeeds.omegaRadiansPerSecond);
  }

  // If the robot is rotating, cancel the rotation override
  if (fieldRelativeSpeeds.omegaRadiansPerSecond > 0 && !RobotState.isAutonomous()) {
    rotationOverridePoint = null;
  }

  // If the robot is in autonomous mode, or if a rotation override point is set, stop the robot from rotating
  if (rotationOverridePoint != null || RobotState.isAutonomous()) {
    fieldRelativeSpeeds.omegaRadiansPerSecond = 0.0;
    if (rotationOverridePoint != null) facePoint(rotationOverridePoint.get(), rotationOverrideOffset);
  }
  
  // Conditionals to compensate for the slop in rotation when aligning with PID controllers
  if (Math.abs(fieldRelativeSpeeds.omegaRadiansPerSecond) > 0.01) {
    setTargetHeading(getHeading());
    isAligning = false;
  }
  if (!isAligning && doneRotating.calculate(Math.abs(getDrivenChassisSpeeds().omegaRadiansPerSecond) < 0.1)) {
    setTargetHeading(getHeading());
    isAligning = true;
  }
  
  // Calculate the angular velocity to align with the target heading
  double alignmentAngularVelocity = alignmentController.calculate(getHeading().getRadians()) + addedAlignmentAngularVelocity;
  addedAlignmentAngularVelocity = 0.0;
  if (isAligning && !alignmentController.atSetpoint() && !parked) fieldRelativeSpeeds.omegaRadiansPerSecond += alignmentAngularVelocity;

  // Calculate the wheel speeds to achieve the desired field-relative speeds
  SwerveDriveWheelStates wheelSpeeds = kinematics.toWheelSpeeds(fieldRelativeSpeeds);
  SwerveDriveKinematics.desaturateWheelSpeeds(
    wheelSpeeds.states,
    fieldRelativeSpeeds,
    SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY,
    SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY,
    SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_VELOCITY
  );
  fieldRelativeSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

  // Limit translational acceleration
  Translation2d targetLinearVelocity = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
  Translation2d currentLinearVelocity = new Translation2d(drivenChassisSpeeds.vxMetersPerSecond, drivenChassisSpeeds.vyMetersPerSecond);
  linearAcceleration = (targetLinearVelocity).minus(currentLinearVelocity).div(Robot.getLoopTime());
  double linearForce = linearAcceleration.getNorm() * SWERVE_DRIVE.ROBOT_MASS;

  // Limit rotational acceleration
  double targetAngularVelocity = fieldRelativeSpeeds.omegaRadiansPerSecond;
  double currentAngularVelocity = drivenChassisSpeeds.omegaRadiansPerSecond;
  angularAcceleration = (targetAngularVelocity - currentAngularVelocity) / Robot.getLoopTime();
  double angularForce = Math.abs((SWERVE_DRIVE.PHYSICS.ROTATIONAL_INERTIA * angularAcceleration) / SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS);
  
  // Limit the total force applied to the robot
  double frictionForce = 9.80 * SWERVE_DRIVE.ROBOT_MASS * SWERVE_DRIVE.FRICTION_COEFFICIENT;
  if (linearForce + angularForce > frictionForce) {
    double factor = (linearForce + angularForce) / frictionForce;
    linearAcceleration = linearAcceleration.div(factor);
    angularAcceleration /= factor;
  }

  // Calculate the attainable linear and angular velocities and update the driven chassis speeds
  Translation2d attainableLinearVelocity = currentLinearVelocity.plus(linearAcceleration.times(Robot.getLoopTime()));
  double attainableAngularVelocity = currentAngularVelocity + (angularAcceleration * Robot.getLoopTime());
  drivenChassisSpeeds = new ChassisSpeeds(attainableLinearVelocity.getX(), attainableLinearVelocity.getY(), attainableAngularVelocity);
  
  // Discretize converts a continous-time chassis speed into discrete-time to compensate for the loop time. This helps reduce drift when rotating while driving in a straight line.
  drivenChassisSpeeds = ChassisSpeeds.discretize(drivenChassisSpeeds, Robot.getLoopTime());
  SwerveDriveWheelStates drivenModuleStates = kinematics.toWheelSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(drivenChassisSpeeds, getAllianceAwareHeading()));
  
  // If the robot is not moving, park the modules
  boolean moving = false;
  for (SwerveModuleState moduleState : kinematics.toWheelSpeeds(fieldRelativeSpeeds).states) if (Math.abs(moduleState.speedMetersPerSecond) > 0.0) moving = true;
  for (SwerveModuleState moduleState : drivenModuleStates.states) if (Math.abs(moduleState.speedMetersPerSecond) > 0.0) moving = true;
  parked = false;

  if (!moving) {
    if (!parkingDisabled) {
      parkModules();
      return;
    }
    // If the robot is aligning, park the modules in a different configuration.
    // Parking normally while aligning can cause the robot to drift away from the setpoint.
    if (alignmentController.atSetpoint()) {
      parkForAlignment();
    }
  }

  parkingDisabled = false;
  driveModules(drivenModuleStates);
}
```

The biggest thing here comes from the acceleration limits we impose on the velocities. By keeping track of the previous velocity, we can calculate our desired acceleration and clamp it to reasonable limits based on the friction force from the carpet.

Additionally, we have a PID controller for rotation. This is to keep the heading of the robot fixed when we're not intending to rotate. This means that if we've been bumped by another robot, we'll automatically re-orient ourselves to face the same direction as before.

### Facing a point
To face a point seems as simple as telling our rotation PID controller to point in a certain direction. Ideally, it would be this simple, but in practice, there is quite a delay between setting the target rotation and the robot actually rotating to that point. If the target direction changes because we're moving, the PID controller won't be able to keep up. This is where we can apply some predictive behavior.

If we know our current velocity, we can calculate the rotational velocity required to stay pointed at the point, and add that to the output of our PID controller. Here's what that whole method looks like:

```java
/**
 * Faces a point on the field
 * @param point
 * @param rotationOffset
 */
public void facePoint(Translation2d point, Rotation2d rotationOffset) {
  double time = 0.02;

  // With no point, do nothing.
  if (point == null) {
    setTargetHeadingAndVelocity(getHeading(), 0.0);
    return;
  }

  // If the robot is close to the point, do nothing.
  if (point.getDistance(getPose().getTranslation()) < 1.0 && RobotState.isAutonomous()) {
    return;
  }

  // Calculate the future position of the robot, and predict how the heading will need to change in the future.
  Translation2d currentPosition = getPose().getTranslation();
  Translation2d futurePosition = getPose().getTranslation().plus(getFieldVelocity().times(time));
  Rotation2d currentTargetHeading = point.minus(currentPosition).getAngle().plus(rotationOffset);
  Rotation2d futureTargetHeading = point.minus(futurePosition).getAngle().plus(rotationOffset);    
  double addedVelocity = futureTargetHeading.minus(currentTargetHeading).getRadians() / time;
  if (getPose().getTranslation().getDistance(point) < 1.0) {
    addedVelocity = 0.0;
  }
  setTargetHeadingAndVelocity(currentTargetHeading, addedVelocity);
}
```

### PathPlanner
Pathplanner is the tool we use for autonomous and teleoperated assisted driving. It allows us to create a path on the field and have the robot follow it. Here's what that setup looks like:

```java
// Path planner setup
AutoBuilder.configureHolonomic(
  this::getPose, // Robot pose supplier
  this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
  this::getMeasuredChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
  this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
  new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    new PIDConstants(SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kP, SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kI, SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kD), // Translation PID constants
    new PIDConstants(SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kP, SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kI, SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kD), // Rotation PID constants
    SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY, // Max module speed, in m/s
    SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
    new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
  ),
  () -> false,
  this // Reference to this subsystem to set requirements
);
```

And when we want to have the robot drive to a position on the field, we can generate a path and have it follow the path like so:

```java
/**
 * Go to a position on the field (without object avoidance)
 * @param pose Field-relative pose on the field to go to
 * @param xboxController Xbox controller to cancel the command
 * @return A command to run
 */
public Command goToSimple(Pose2d pose) {
  Rotation2d angle = pose.getTranslation().minus(getPose().getTranslation()).getAngle();

  List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
    new Pose2d(getPose().getTranslation(), angle),
    new Pose2d(pose.getTranslation(), angle)
  );

  PathPlannerPath path = new PathPlannerPath(
    bezierPoints,
    SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
    new GoalEndState(
      0.0,
      pose.getRotation(),
      true
    )
  );

  return Commands.sequence(
    AutoBuilder.followPath(path),
    runOnce(() -> setTargetHeading(pose.getRotation()))
  );
}
```

If we want to do something more complicated, we can avoid objects with pathfinding:

```java
/**
 * Pathfind to a first given point on the field, and then follow a straight line to the second point.
 * @param firstPoint
 * @param secondPoint
 * @return
 */
public Command pathfindThenFollowPath(Pose2d firstPoint, Pose2d secondPoint) {
  Rotation2d angle = secondPoint.getTranslation().minus(firstPoint.getTranslation()).getAngle();

  List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
    new Pose2d(firstPoint.getTranslation(), angle),
    new Pose2d(secondPoint.getTranslation(), angle)
  );

  PathPlannerPath path = new PathPlannerPath(
    bezierPoints,
    SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
    new GoalEndState(
      0.0,
      secondPoint.getRotation(),
      true
    )
  );
  
  return AutoBuilder.pathfindThenFollowPath(
    path,
    SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
    0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );
}
```