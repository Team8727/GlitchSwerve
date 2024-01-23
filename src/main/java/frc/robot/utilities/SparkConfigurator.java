package frc.robot.utilities;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
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

    int status0 = FAST; // Applied Output & Faults
    int status1 = SLOW; // Velocity, Voltage, & Current
    int status2 = SLOW; // Position
    int status3 = OFF; // Analog Sensor
    int status4 = OFF; // Alternate Encoder
    int status5 = OFF; // Absolute Encoder Position
    int status6 = OFF; // Absolute Encoder Velocity

    if (!hasFollower && !logData.contains(LogData.VOLTAGE)) {
      status0 = SLOW;
    }

    if (logData.contains(LogData.VELOCITY)
        || logData.contains(LogData.VOLTAGE)
        || logData.contains(LogData.CURRENT)) {
      status1 = NORMAL;
    }

    if (logData.contains(LogData.POSITION)) status2 = NORMAL;

    if (sensors.contains(Sensors.ANALOG)) status3 = NORMAL;

    if (sensors.contains(Sensors.ALTERNATE)) status4 = NORMAL;

    if (sensors.contains(Sensors.ABSOLUTE)) {
      if (logData.contains(LogData.POSITION)) status5 = NORMAL;
      if (logData.contains(LogData.VELOCITY)) status6 = NORMAL;
    }

    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, status0);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, status1);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, status2);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, status3);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, status4);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, status5);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, status6);

    return spark;
  }

  public static CANSparkMax getFollower(CANSparkMax leader, int id, MotorType motorType) {
    CANSparkMax spark = new CANSparkMax(id, motorType);
    spark.follow(leader);

    int status0 = SLOW;
    int status1 = SLOW;
    int status2 = SLOW;
    int status3 = OFF;
    int status4 = OFF;
    int status5 = OFF;
    int status6 = OFF;

    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, status0);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, status1);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, status2);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, status3);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, status4);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, status5);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, status6);

    return spark;
  }
}
