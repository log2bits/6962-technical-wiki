# Simulation
Another key factor in reducing debug time and enabling faster code development is simulation support. The biggest bottleneck for software development is time available on the robot, and most seasons, the code team only gets a few weeks near the end of the season to work on the robot, even if it isn't fully working. Simulation not only allows for code development much earlier, but also allows for development from home or really anywhere with a computer.

Swerve drive is the most complicated mechanism to simulate, so I'll talk about it here. The key to good simulation is making sure that its implemented at as low a level as possible. If the simulation is done all the way down to the motor level (as it should), then the rest of the code should not need to be changed, and all code that would normally control the robot, would control the simulation in the same exact way. This allows for the same code to work in both scenarios with no changes whatsoever.

To implement this system for swerve drive, we create a simulated swerve module that inherits all the functionality from the real swerve module class:

```java
public class SwerveModuleSim extends SwerveModule {
  private FlywheelSim driveMotor = new FlywheelSim(
    LinearSystemId.identifyVelocitySystem(DRIVE_MOTOR_PROFILE.kV * SWERVE_DRIVE.WHEEL_RADIUS, DRIVE_MOTOR_PROFILE.kA * SWERVE_DRIVE.WHEEL_RADIUS),
    NEO.STATS,
    SWERVE_DRIVE.DRIVE_MOTOR_GEARING
  );
  
  private FlywheelSim steerMotor = new FlywheelSim(
    LinearSystemId.identifyVelocitySystem(STEER_MOTOR_PROFILE.kV, STEER_MOTOR_PROFILE.kA),
    NEO.STATS,
    SWERVE_DRIVE.STEER_MOTOR_GEARING
  );

  private PIDController drivePID = new PIDController(
    DRIVE_MOTOR_PROFILE.kP,
    DRIVE_MOTOR_PROFILE.kI,
    DRIVE_MOTOR_PROFILE.kD
  );
  private PIDController steerPID = new PIDController(
    STEER_MOTOR_PROFILE.kP, 
    STEER_MOTOR_PROFILE.kI,
    STEER_MOTOR_PROFILE.kD
  );
  private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(
    DRIVE_MOTOR_PROFILE.kS,
    DRIVE_MOTOR_PROFILE.kV,
    DRIVE_MOTOR_PROFILE.kA
  );
      
  private double driveVoltRamp = 0.0;
  private double steerVoltRamp = 0.0;
  private double drivePosition = 0.0;
  private double steerRadians = (Math.random() * 2.0 * Math.PI) - Math.PI;
  
  public SwerveModuleSim(MODULE_CONFIG config, int corner, String name) {
    super(config, corner, name);
    steerPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void drive(SwerveModuleState state) {
    double speedMetersPerSecond = state.speedMetersPerSecond;
    double radians = state.angle.getRadians();

    if (SWERVE_DRIVE.DO_ANGLE_ERROR_SPEED_REDUCTION) {
      speedMetersPerSecond *= Math.cos(SwerveMath.angleDistance(getMeasuredState().angle.getRadians(), getMeasuredState().angle.getRadians()));
    }
    
    for (int i = 0; i < 20; i++) {
      double driveVolts = driveFF.calculate(speedMetersPerSecond, 0.0) + 12.0 * drivePID.calculate(getMeasuredState().speedMetersPerSecond, speedMetersPerSecond);
      double steerVolts = 12.0 * steerPID.calculate(getMeasuredState().angle.getRadians(), radians);
      
      driveVoltRamp += (MathUtil.clamp(driveVolts - driveVoltRamp, -12.0 / (SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY / SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION) / 1000.0, 12.0 / (SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY / SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION) / 1000.0));
      driveVolts = driveVoltRamp;
      
      steerVoltRamp += (MathUtil.clamp(steerVolts - steerVoltRamp, -12.0 / NEO.SAFE_RAMP_RATE / 1000.0, 12.0 / NEO.SAFE_RAMP_RATE / 1000.0));
      steerVolts = steerVoltRamp;

      driveMotor.setInputVoltage(MathUtil.clamp(driveVolts, -12.0, 12.0));
      steerMotor.setInputVoltage(MathUtil.clamp(steerVolts, -12.0, 12.0));

      driveMotor.update(1.0 / 1000.0);
      steerMotor.update(1.0 / 1000.0);

      drivePosition += getMeasuredState().speedMetersPerSecond * (1.0 / 1000.0);
      steerRadians += steerMotor.getAngularVelocityRadPerSec() * (1.0 / 1000.0);
      steerRadians = MathUtil.angleModulus(steerRadians);
    }
  }

  @Override
  public void seedSteerEncoder() {
    return;
  }
  
  @Override
  public void stop() {
    super.setTargetState(new SwerveModuleState(0.0, getMeasuredState().angle));
    steerMotor.setInputVoltage(0.0);
    driveMotor.setInputVoltage(0.0);
  }

  @Override
  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(driveMotor.getAngularVelocityRadPerSec() * SWERVE_DRIVE.WHEEL_RADIUS, Rotation2d.fromRadians(steerRadians));
  }

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(drivePosition, getMeasuredState().angle);
  }
}
```

The only difference between this swerve module implementation and the original is the encoders, motors, and on-board PID controllers, since those are the only physical devices that we lack in software.

The motors are represented as flywheels and are given the appropriate gearing and motor specs:

```java
private FlywheelSim driveMotor = new FlywheelSim(
  LinearSystemId.identifyVelocitySystem(DRIVE_MOTOR_PROFILE.kV * SWERVE_DRIVE.WHEEL_RADIUS, DRIVE_MOTOR_PROFILE.kA * SWERVE_DRIVE.WHEEL_RADIUS),
  NEO.STATS,
  SWERVE_DRIVE.DRIVE_MOTOR_GEARING
);

private FlywheelSim steerMotor = new FlywheelSim(
  LinearSystemId.identifyVelocitySystem(STEER_MOTOR_PROFILE.kV, STEER_MOTOR_PROFILE.kA),
  NEO.STATS,
  SWERVE_DRIVE.STEER_MOTOR_GEARING
);
```

With these `FlywheelSim` objects that come with WPILib, we can simulate a voltage with:

```java
driveMotor.setInputVoltage(voltage);
```

And measure the velocity with:

```java
driveMotor.getAngularVelocityRadPerSec();
```

All of the PID control is done with WPILib's `PIDController` class, but to simulate the on-board PID control on the SparkMAX's at 1000 Hz, we run the control loop 20 times every 50 ms:

```java
for (int i = 0; i < 20; i++) {
  double driveVolts = driveFF.calculate(speedMetersPerSecond, 0.0) + 12.0 * drivePID.calculate(getMeasuredState().speedMetersPerSecond, speedMetersPerSecond);
  double steerVolts = 12.0 * steerPID.calculate(getMeasuredState().angle.getRadians(), radians);
    
  driveVoltRamp += (MathUtil.clamp(driveVolts - driveVoltRamp, -12.0 / (SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY / SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION) / 1000.0, 12.0 / (SWERVE_DRIVE.PHYSICS  MAX_LINEAR_VELOCITY / SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION) / 1000.0));
  driveVolts = driveVoltRamp;
    
  steerVoltRamp += (MathUtil.clamp(steerVolts - steerVoltRamp, -12.0 / NEO.SAFE_RAMP_RATE / 1000.0, 12.0 / NEO.SAFE_RAMP_RATE / 1000.0));
  steerVolts = steerVoltRamp;

  driveMotor.setInputVoltage(MathUtil.clamp(driveVolts, -12.0, 12.0));
  steerMotor.setInputVoltage(MathUtil.clamp(steerVolts, -12.0, 12.0));

  driveMotor.update(1.0 / 1000.0);
  steerMotor.update(1.0 / 1000.0);

  drivePosition += getMeasuredState().speedMetersPerSecond * (1.0 / 1000.0);
  steerRadians += steerMotor.getAngularVelocityRadPerSec() * (1.0 / 1000.0);
  steerRadians = MathUtil.angleModulus(steerRadians);
}
```

And that's it! The swerve module simulation will now emulate real-world motor physics and you can drive around in simulation!