package frc.robot.utilities;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import java.util.Set;

public class SparkConfigurator {
  // Frame speeds in ms
  private static final int FAST = 10;
  private static final int NORMAL = 20;
  private static final int SLOW = 200;
  private static final int OFF = 65535;

  // Sensor options
  public enum Sensors {
    INTEGRATED,
    ABSOLUTE,
    ALTERNATE,
    ANALOG
  }

  // Data logging options
  public enum LogData {
    VOLTAGE,
    CURRENT,
    POSITION,
    VELOCITY
  }

  // Get a sparkmax with no follower, sensors, or logged data
  public static CANSparkMax getSparkMax(int id, MotorType motorType) {
    return getSparkMax(id, motorType, false, Set.of(), Set.of());
  }

  // Get a sparkmax with no sensors or logged data
  public static CANSparkMax getSparkMax(int id, MotorType motorType, boolean hasFollower) {
    return getSparkMax(id, motorType, hasFollower, Set.of(), Set.of());
  }

  // Get a sparkmax
  public static CANSparkMax getSparkMax(
      int id,
      MotorType motorType,
      boolean hasFollower,
      Set<Sensors> sensors,
      Set<LogData> logData) {
    CANSparkMax spark = new CANSparkMax(id, motorType);
    spark.restoreFactoryDefaults();

    int[] status = {FAST, SLOW, SLOW, OFF, OFF, OFF, OFF};
    // status0 Applied Output & Faults
    // status1 Velocity, Voltage, & Current
    // status2 Position
    // status3 Analog Sensor
    // status4 Alternate Encoder
    // status5 Absolute Encoder Position
    // status6 Absolute Encoder Velocity

    if (!hasFollower && !logData.contains(LogData.VOLTAGE)) {
      status[0] = SLOW;
    }

    if (logData.contains(LogData.VELOCITY)
        || logData.contains(LogData.VOLTAGE)
        || logData.contains(LogData.CURRENT)) {
      status[1] = NORMAL;
    }

    if (logData.contains(LogData.POSITION)) status[2] = NORMAL;

    if (sensors.contains(Sensors.ANALOG)) status[3] = NORMAL;

    if (sensors.contains(Sensors.ALTERNATE)) status[4] = NORMAL;

    if (sensors.contains(Sensors.ABSOLUTE)) {
      if (logData.contains(LogData.POSITION)) status[5] = NORMAL;
      if (logData.contains(LogData.VELOCITY)) status[6] = NORMAL;
    }

    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < Constants.configurationSetRetries; j++) {
        spark.setPeriodicFramePeriod(PeriodicFrame.values()[i], status[i]);
      }
    }

    return spark;
  }

  public static CANSparkMax getFollower(
      CANSparkMax leader, int id, MotorType motorType, boolean invert) {
    CANSparkMax spark = new CANSparkMax(id, motorType);
    spark.follow(leader, invert);

    int[] status = {SLOW, SLOW, SLOW, OFF, OFF, OFF, OFF};
    // status0 Applied Output & Faults
    // status1 Velocity, Voltage, & Current
    // status2 Position
    // status3 Analog Sensor
    // status4 Alternate Encoder
    // status5 Absolute Encoder Position
    // status6 Absolute Encoder Velocity

    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < Constants.configurationSetRetries; j++) {
        spark.setPeriodicFramePeriod(PeriodicFrame.values()[i], status[i]);
        try {
          Thread.sleep(5);
        } catch (Exception e) {
        }
      }
    }

    return spark;
  }
}
