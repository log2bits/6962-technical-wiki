# Telemetry
Telemetry is one of the most important and yet simple things that can really accelerate the debug process. Telemetry is all about collecting data about the robot and saving it to logs to view later. With sufficiently advanced software, we can replay an entire match from log files, and observe where the robot failed, and speculate why. 

The tool that we've to replay log files used is called [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope), and is extremely powerful.

Despite these powerful tools, the important part is the logging itself. Software needs to exist to log the values you'd like to save, and that's what I'll talk about now.

## NetworkTables
A very simple and powerful method to log data is using networktables. You can imagine that the `NetworkTable` is just a big table full of data that is sent back and forth from the drive computer and the robot from the radio. This table contains anything you'd like, you just need to write to it.

This is as simple as:
```java
NetworkTable table = NetworkTableInstance.getDefault().getTable("Logs");
table.getEntry(key).setValue(obj);
```

This allows logs to be looked at in real-time from the drive computer, but doesnt save anything to a file for viewing later. This is where the `DataLogManager` comes into play.

## DataLogManager
By calling this at the start of the robot program,

```java
DataLogManager.start();
```

Everything that is sent over network tables will be saved to a .wpilog file either on the roboRIO or a connected usb stick. This log file contains every entry to the network tables and a timestamp on each, so it can be replayed exactly as it was recorded.

Additionally, to log data from the driver station, you can call:

```java
DriverStation.startDataLog(DataLogManager.getLog(), true);
```

## Custom Logger

Now to log data, all we need to do is put values into the network tables. This is simple, but can get quite annoying with how many entries we might want to log, so we create our own custom logging class:

```java
public final class Logger {
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("Logs");
  private static Map<String, Supplier<Object>> suppliers = new HashMap<String, Supplier<Object>>();
  private static Map<String, Object> values = new HashMap<String, Object>();
  private static ShuffleboardTab tab = Shuffleboard.getTab("Logging");
  private static GenericEntry loggingButton = tab.add("Enable Logging", true).withWidget(BuiltInWidgets.kToggleSwitch).withSize(1, 1).withPosition(0, 0).getEntry();
  
  private static Notifier notifier = new Notifier(
    () -> {
      try {
        Logger.logAll();
      } catch (Exception e) {
        
      }
    }
  );
  
  public static void startLog() {
    notifier.startPeriodic(LOGGING.LOGGING_PERIOD_MS / 1000.0);
  }

  public static boolean loggingEnabled() {
    return loggingButton.getBoolean(false);
  }

  private static void logAll() {
    if (!loggingButton.getBoolean(false)) return;

    for (String key : suppliers.keySet()) {
      Object supplied_value = suppliers.get(key).get();
      Object saved_value = values.get(key);
      if (supplied_value.equals(saved_value)) {
        continue;
      }
      try {
        log(key, supplied_value);
      } catch (IllegalArgumentException e) {
        System.out.println("[LOGGING] unknown type: " + supplied_value.getClass().getSimpleName());
      }
    }
    logRio("roboRio");
  }

  public static void autoLog(String key, Supplier<Object> supplier) {
    suppliers.put(key, supplier);
  }

  public static void autoLog(String key, Object obj) {
    autoLog(key, () -> obj);
  }

  public static void autoLog(SubsystemBase subsystem, String key, Supplier<Object> supplier) {
    autoLog(subsystem.getClass().getSimpleName() + "/" + key, supplier);
  }

  public static void log(String key, Object obj) {
    if (obj instanceof CANSparkMax) log(key, (CANSparkMax) obj);
    else if (obj instanceof RelativeEncoder) log(key, (RelativeEncoder) obj);
    else if (obj instanceof AHRS) log(key, (AHRS) obj);
    else if (obj instanceof Pose2d) log(key, (Pose2d) obj);
    else if (obj instanceof SwerveModuleState) log(key, (SwerveModuleState) obj);
    else if (obj instanceof SwerveModuleState[]) log(key, (SwerveModuleState[]) obj);
    else if (obj instanceof SwerveModulePosition[]) log(key, (SwerveModulePosition[]) obj);
    else if (obj instanceof CANStatus) log(key, (CANStatus) obj);
    else if (obj instanceof PowerDistribution) log(key, (PowerDistribution) obj);
    else if (obj instanceof Translation2d) log(key, (Translation2d) obj);
    else if (obj instanceof Translation3d) log(key, (Translation3d) obj);
    else {
      table.getEntry(key).setValue(obj);
      values.put(key, obj);
    };
  }

  public static void log(String path, Translation3d translation) {
    log(path, new double[] {translation.getX(), translation.getY(), translation.getZ()});
  }

  public static void log(String path, Translation2d translation) {
    log(path, new double[] {translation.getX(), translation.getY()});
  }

  public static void log(String path, CANcoder encoder) {
    log(path + "/absolutePosition", encoder.getAbsolutePosition());
    log(path + "/position", encoder.getPosition());
    log(path + "/velocity", encoder.getVelocity());
  }

  public static void log(String path, AHRS navX) {
    log(path + "/isAltitudeValid", navX.isAltitudeValid());
    log(path + "/isCalibrating", navX.isCalibrating());
    log(path + "/isConnected", navX.isConnected());
    log(path + "/isMagneticDisturbance", navX.isMagneticDisturbance());
    log(path + "/isMagnetometerCalibrated", navX.isMagnetometerCalibrated());
    log(path + "/isMoving", navX.isMoving());
    log(path + "/isRotating", navX.isRotating());
    log(path + "/actualUpdateRate", navX.getActualUpdateRate());
    log(path + "/firmwareVersion", navX.getFirmwareVersion());
    log(path + "/altitude", navX.getAltitude());
    log(path + "/angle", navX.getAngle());
    log(path + "/angleAdjustment", navX.getAngleAdjustment());
    log(path + "/compassHeading", navX.getCompassHeading());
    log(path + "/displacementX", navX.getDisplacementX());
    log(path + "/displacementY", navX.getDisplacementY());
    log(path + "/displacementZ", navX.getDisplacementZ());
    log(path + "/fusedHeading", navX.getFusedHeading());
    log(path + "/pitch", navX.getPitch());
    log(path + "/pressure", navX.getPressure());
    log(path + "/roll", navX.getRoll());
    log(path + "/yaw", navX.getYaw());
    log(path + "/temperature", navX.getTempC());
    log(path + "/velocityX", navX.getVelocityX());
    log(path + "/velocityY", navX.getVelocityY());
    log(path + "/velocityZ", navX.getVelocityZ());
    log(path + "/accelerationX", navX.getRawAccelX());
    log(path + "/accelerationY", navX.getRawAccelY());
    log(path + "/accelerationZ", navX.getRawAccelZ());
  }

  public static void log(String path, Pose2d pose) {
    log(path + "_radians", new double[] { pose.getX(), pose.getY(), pose.getRotation().getRadians() });
    log(path + "_degrees", new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() });
  }

  public static void log(String path, SwerveModuleState swerveModuleState) {
    log(path + "/state", new double[] {
      swerveModuleState.angle.getRadians(), 
      swerveModuleState.speedMetersPerSecond
    });
  }

  public static void log(String path, SwerveModuleState[] swerveModuleStates) {
    log(path + "/states", new double[] {
      swerveModuleStates[0].angle.getRadians(), 
      swerveModuleStates[0].speedMetersPerSecond, 
      swerveModuleStates[1].angle.getRadians(), 
      swerveModuleStates[1].speedMetersPerSecond, 
      swerveModuleStates[2].angle.getRadians(), 
      swerveModuleStates[2].speedMetersPerSecond, 
      swerveModuleStates[3].angle.getRadians(), 
      swerveModuleStates[3].speedMetersPerSecond, 
    });
  }

  public static void log(String path, SwerveModulePosition[] swerveModulePositions) {
    log(path + "/positions", new double[] {
      swerveModulePositions[0].angle.getRadians(), 
      swerveModulePositions[0].distanceMeters, 
      swerveModulePositions[1].angle.getRadians(), 
      swerveModulePositions[1].distanceMeters, 
      swerveModulePositions[2].angle.getRadians(), 
      swerveModulePositions[2].distanceMeters, 
      swerveModulePositions[3].angle.getRadians(), 
      swerveModulePositions[3].distanceMeters, 
    });
  }

  public static void logRio(String path) {
    log(path + "/isBrownedOut", RobotController.isBrownedOut());
    log(path + "/isSysActive", RobotController.isSysActive());
    log(path + "/brownoutVoltage", RobotController.getBrownoutVoltage());
    log(path + "/batteryVoltage", RobotController.getBatteryVoltage());
    log(path + "/batteryVoltage", RobotController.getBatteryVoltage());
    log(path + "/inputCurrent", RobotController.getInputCurrent());
    log(path + "/inputVoltage", RobotController.getInputVoltage());
    log(path + "/3V3Line/current", RobotController.getCurrent3V3());
    log(path + "/5VLine/current", RobotController.getCurrent5V());
    log(path + "/6VLine/current", RobotController.getCurrent6V());
    log(path + "/3V3Line/enabled", RobotController.getEnabled3V3());
    log(path + "/5VLine/enabled", RobotController.getEnabled5V());
    log(path + "/6VLine/enabled", RobotController.getEnabled6V());
    log(path + "/3V3Line/faultCount", RobotController.getFaultCount3V3());
    log(path + "/5VLine/faultCount", RobotController.getFaultCount5V());
    log(path + "/6VLine/faultCount", RobotController.getFaultCount6V());
    log(path + "/3V3Line/voltage", RobotController.getVoltage3V3());
    log(path + "/5VLine/voltage", RobotController.getVoltage5V());
    log(path + "/6VLine/voltage", RobotController.getVoltage6V());
    log(path + "/canStatus", RobotController.getCANStatus());
  }

  public static void log(String path, CANStatus canStatus) {
    log(path + "/busOffCount", canStatus.busOffCount);
    log(path + "/percentBusUtilization", canStatus.percentBusUtilization);
    log(path + "/receiveErrorCount", canStatus.receiveErrorCount);
    log(path + "/transmitErrorCount", canStatus.transmitErrorCount);
    log(path + "/txFullCount", canStatus.txFullCount);
  }

  public static void log(String path, PowerDistribution PDH) {
    log(path + "/faults", PDH.getFaults());
    log(path + "/canId", PDH.getModule());
    for (int i = 0; i <= 23; i++) {
      log(path + "/channels/channel" + i + "Current", PDH.getCurrent(i));
    }
    log(path + "/isSwitchableChannelOn", PDH.getSwitchableChannel());
    log(path + "/temperature", PDH.getTemperature());
    log(path + "/totalCurrent", PDH.getTotalCurrent());
    log(path + "/totalJoules", PDH.getTotalEnergy());
    log(path + "/totalWatts", PDH.getTotalPower());
    log(path + "/voltage", PDH.getVoltage());
  }

  public static void log(String path, PowerDistributionFaults faults) {
    log(path + "/brownout", faults.Brownout);
    log(path + "/canWarning", faults.CanWarning);

    for (int i = 0; i < 24; i++) {
      log(path + "/channel" + i + "BreakerFault", faults.getBreakerFault(i));
    }
    
    log(path + "/hardwareFault", faults.HardwareFault);
  }

  public static void log(String path, Object self, Class clazz) {    
    for (Class c: clazz.getDeclaredClasses()) {
      try {
        log(path + "/" + c.getSimpleName(), self, c);
      }
      catch (Exception e) {}
    }

    for (Field f: clazz.getDeclaredFields()) {
      try {
        log(path + "/" + f.getName(), f.get(self));
      }
      catch (Exception e) {}
    }
  }
}
```

You might notice that there are a lot of methods with the same name, notably the `log` method. This is called "method overloading" and allows only the method with the right arguments to be used whenever calling it. This means that we can log any datatype, as long as we have some hardcoded types that we allow. Thats this section:

```java
public static void log(String key, Object obj) {
  if (obj instanceof CANSparkMax) log(key, (CANSparkMax) obj);
  else if (obj instanceof RelativeEncoder) log(key, (RelativeEncoder) obj);
  else if (obj instanceof AHRS) log(key, (AHRS) obj);
  else if (obj instanceof Pose2d) log(key, (Pose2d) obj);
  else if (obj instanceof SwerveModuleState) log(key, (SwerveModuleState) obj);
  else if (obj instanceof SwerveModuleState[]) log(key, (SwerveModuleState[]) obj);
  else if (obj instanceof SwerveModulePosition[]) log(key, (SwerveModulePosition[]) obj);
  else if (obj instanceof CANStatus) log(key, (CANStatus) obj);
  else if (obj instanceof PowerDistribution) log(key, (PowerDistribution) obj);
  else if (obj instanceof Translation2d) log(key, (Translation2d) obj);
  else if (obj instanceof Translation3d) log(key, (Translation3d) obj);
  else {
    table.getEntry(key).setValue(obj);
    values.put(key, obj);
  };
}
```

The magical part of this logging class is the `autoLog` methods. These methods take in a supplier to the value we want to log, and automatically log the value periodically. On top of that, one of the `autoLog` methods takes in the subsystem that the data is from, and will automatically create a key that is tied to the subsystem's name, so that our log files have a neat tree structure.

This means that logging becomes as easy as:

```java
Logger.autoLog(this, "Is Aimed", () -> isAimed());
```

And thats it! No messing around with periodic loops or super long keys, just call that and you don't need to worry about it anymore.

## Status Checks
Another important part of telemetry is making sure systems are functioning properly. If not, someone should know about it, and that's where status checks come in. There is a panel on the dashboard full of green lights to indicate different status checks, and if some are red, there is clearly a problem. This is done using the `StatusChecks` class:

```java
public class StatusChecks {
  public static int row = 0;
  public static int column = 1;
  private static ShuffleboardTab tab = Shuffleboard.getTab("Status Checks");
  private static ComplexWidget refreshButton = tab.add("Refresh", StatusChecks.refresh()).withWidget(BuiltInWidgets.kCommand).withSize(1, 1).withPosition(0, 0);
  private static Map<String, BooleanSupplier> suppliers = new HashMap<String, BooleanSupplier>();
  private static Map<String, GenericEntry> entries = new HashMap<String, GenericEntry>();

  private static void addCheck(String name, BooleanSupplier supplier) {
    suppliers.put(name, supplier);
    entries.put(name, tab.add(name.replace("/", " "), supplier.getAsBoolean()).withWidget(BuiltInWidgets.kBooleanBox).withSize(1, 1).withPosition(column, row).getEntry());
    column++;
    if (column > 12) {
      column = 0;
      row++;
    }
  }

  public static void addCheck(SubsystemBase subsystem, String name, BooleanSupplier supplier) {
    addCheck(subsystem.getClass().getSimpleName() + "/" + name, supplier);
    
  }

  public static Command refresh() {
    return Commands.runOnce(() -> {
      for (Map.Entry<String, BooleanSupplier> supplier : suppliers.entrySet()) {
        entries.get(supplier.getKey()).setBoolean(supplier.getValue().getAsBoolean());
      }
    }).ignoringDisable(true);
  }
}
```

To add a status check, call it like so:

```java
StatusChecks.addCheck(this, "isGyroConnected", () -> gyro.isConnected());
```

And it'll be automatically added to the dashboard.