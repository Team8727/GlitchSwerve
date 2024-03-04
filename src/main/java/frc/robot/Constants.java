// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;

public class Constants {
  public static SimMode simMode = SimMode.DESKTOP;
  public static TestMode testMode = TestMode.NONE;
  public static boolean logFileOnly = false;
  public static boolean logLazy = true;
  public static int configurationSetRetries = 5;

  // Operator interface constants
  public static class kOI {
    public static double translationDeadzone = 0.08;
    public static double rotationDeadzone = 0.08;
  }

  // Sim Modes
  public enum SimMode {
    HARDWARE_IN_LOOP,
    DESKTOP_VISION,
    DESKTOP
  }

  // SysId
  public static enum sysIdType {
    LINEAR,
    ANGULAR
  }

  public static enum subsystems {
    SWERVE,
    INTAKEPIVOT,
    INTAKEROLLERS,
    SHOOTERFLYWHEELS,
    SHOOTERPIVOT,
    CLIMBER
  }

  public static class kClimber {
    public static int climberID = 15;

    // feedForward constants below. This is used in initializing feedForward objects: [Object Name]
    // = SimpleMotorFeedforward(kS, kV, kA);
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // PID values of P and D
    public static final double kP = 0;
    public static final double kD = 0;

    // Current limit of the Climber SparkMax (climbMotor)
    public static final double currentLimit = 0;

    // Parameters for the filter used in getting the current output of climbMotor
    public static final double timeConstant = 0;
    public static final double period = 0.02;

    public static final double rotationsToClimb = 0;
  }

  public enum TestMode {
    NONE,
    SYSID,
    NO_BRAKE_MODE
  }

  // Swerve subsystem constants (module constants included)
  public static class kSwerve {
    // Chassis dimensions from wheel center to center (meters)
    public static double width = Units.inchesToMeters(23);
    public static double length = width;

    // Speed & accel limits (m/s, rad/s, m/s^2 & rad/s^2)
    public static double maxTransSpeed = 5;
    public static double maxAngSpeed = 3 * Math.PI;

    public static double maxTransAccel = 1.25 * 9.81;
    public static double maxAngAccel = 10 * 2 * Math.PI;

    // Operator interface constants
    public static class Teleop {
      public static double translationGain = 0.85;
      public static double rotationGain = 0.7;

      public static boolean closedLoop = false;
    }

    // NavX
    public static boolean invertGyro = false;
    public static Port navxPort = Port.kMXP;

    // Swerve uses ccw+ angular quanities and a coordinate plane with 0,0 at the robot's center
    // , forward is +x, and a module order based on the quadrant system (front left is first)
    public static SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(length / 2, width / 2),
            new Translation2d(-length / 2, width / 2),
            new Translation2d(-length / 2, -width / 2),
            new Translation2d(length / 2, -width / 2));

    // Module angular offsets (rad)
    public static class Offsets {
      public static double frontLeft = 0;
      public static double backLeft = Math.PI / 2;
      public static double backRight = Math.PI;
      public static double frontRight = -Math.PI / 2;
    }

    // Controller PID values for x/y translation, and z rotation
    public static class Auton {
      public static final double angP = 4;
      public static final double angD = 0;

      public static final double maxAccel = 1.5;
      public static final double maxVel = 1.5;
      public static final double maxAngAccel = 0.75 * kSwerve.maxAngAccel;
      public static final double maxAngVel = 0.75 * kSwerve.maxAngSpeed;

      public static final HolonomicPathFollowerConfig pathFollowConfig =
          new HolonomicPathFollowerConfig(
              new PIDConstants(12, 0.0, 0), // Translation PID constants
              new PIDConstants(angP, 0.0, angD), // Rotation PID constants
              kModule.maxWheelSpeed, // Max module speed, in m/s
              Math.pow(
                  Math.pow(kSwerve.width / 2, 2) + Math.pow(kSwerve.length / 2, 2),
                  1 / 2), // Drive base radius in meters. Distance from robot center to furthest
              // module.
              new ReplanningConfig(false, false));
    }

    public static class kModule {
      // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
      public static final int drivePinionTeeth = 14;
      public static final boolean invertSteerEncoder = true;

      // Controls Constants
      public static class kDrive {
        public static final double kP = 0.2;
        public static final double kD = 0.05;
        public static final double kS = 0.068841;
        public static final double kV = 2.4568;
        public static final double kA = 0.22524;
        public static final double minOutput = -1;
        public static final double maxOutput = 1;
      }

      public static class kSteer {
        public static final double kP = 3;
        public static final double kD = 0;
        public static final double minOutput = -1;
        public static final double maxOutput = 1;
      }

      // Motor configs
      public static final IdleMode drivingMotorIdleMode = IdleMode.kBrake;
      public static final IdleMode steeringMotorIdleMode = IdleMode.kBrake;

      public static final int driveSmartCurrentLimit = 50; // amps
      public static final int driveMaxCurrent = 80; // amps
      public static final int steerSmartCurrentLimit = 20; // amps
      public static final int steerMaxCurrent = 35; // amps

      // Physical dimensions/values
      public static final double wheelDiameter = Units.inchesToMeters(3);
      public static final double wheelCircumference = wheelDiameter * Math.PI; // meters
      public static final double driveMotorReduction = (45.0 * 22) / (drivePinionTeeth * 15);
      public static final double steerMotorReduction = 9424.0 / 203.0;

      // Motor physics
      public static final double neoFreeSpeed = 5820.0 / 60; // rot/s
      public static final double neoFreeCurrent = 1.7; // amps
      public static final double neoStallTorque = 3.28; // Nm
      public static final double neoStallCurrent = 181; // amps

      public static final double neo550FreeSpeed = 11710.0 / 60;
      public static final double neo550FreeCurrent = 1.1;
      public static final double neo550StallTorque = 1.08;
      public static final double neo550StallCurrent = 111;

      public static final double maxWheelSpeed =
          (neoFreeSpeed / driveMotorReduction) * (wheelDiameter * Math.PI); // m/s

      // Encoders
      public static final double drivingEncoderPositionFactor =
          (wheelDiameter * Math.PI) / driveMotorReduction; // meters
      public static final double drivingEncoderVelocityFactor =
          ((wheelDiameter * Math.PI) / driveMotorReduction) / 60.0; // m/s

      public static final double steeringEncoderPositionFactor = (2 * Math.PI); // radians
      public static final double steeringEncoderVelocityFactor = (2 * Math.PI) / 60.0; // rad/s

      public static final double steeringEncoderPositionPIDMinInput = 0; // radians
      public static final double steeringEncoderPositionPIDMaxInput =
          steeringEncoderPositionFactor; // radians
    }

    // Motor CAN IDs
    public static class CANID {
      public static int frontLeftDrive = 1;
      public static int frontLeftSteer = 2;
      public static int backLeftDrive = 7;
      public static int backLeftSteer = 8;
      public static int backRightDrive = 5;
      public static int backRightSteer = 6;
      public static int frontRightDrive = 3;
      public static int frontRightSteer = 4;
    }

    // Vision
    public static final Transform3d aprilTagCamera1PositionTransform =
        new Transform3d(
            new Translation3d(0.243, 0.193, 0.229),
            new Rotation3d(0, 0, 0)); // TODO GET REAL ONE this is from last year
    public static final Transform3d aprilTagCamera2PositionTransform =
        new Transform3d(
            new Translation3d(0.243, 0.193, 0.229),
            new Rotation3d(0, 0, 0)); // TODO GET REAL ONE this is from last year
  }

  public static class kIntake {
    public static class kPivot {
      public static int pivotMotorID = 10;
      public static double period = 0.02;
      public static double minPIDOutput = -1.0;
      public static double maxPIDOutput = 1.0;
      // PID
      public static double kP = 4;
      public static double kI = 0;
      public static double kD = 0.015;
      // FF
      // public static double kS = 0.4273;
      public static double kS = 0.15;
      public static double kG = 0.23;
      public static double kV = 1.1;
      public static double kA = 0.24565;

      // ProfiledPIDController
      public static double maxVel = 8;
      public static double maxAccel = 40;

      // Encoder
      public static int portA = 3;
      public static int portB = 2;
      public static double pulsesPerRevolution = 2048;
      public static Rotation2d cogOffset = new Rotation2d(Math.PI);
      public static boolean invertedEncoder = false;

      public static final double resetProfiledPIDControllerPos = 0;
      public static final Rotation2d intakeRadiansDown = new Rotation2d(0);
      public static final Rotation2d intakeRadiansHome = new Rotation2d(Math.PI);
    }

    public static class kRollers {
      public static int rollerMotorID = 9;
      public static int sensorChannel = 0; // TODO get real channel
      public static double intakeVoltage = 1; // TODO Tune
      public static double outtakeVoltage = -1; // TODO tune
      public static double delayForOuttake = 1;
    }
  }

  public static class kShooter {
    public static class kPivot {
      public static int pivot1MotorID = 11;
      public static int pivot2MotorID = 12;

      // PD
      public static double kP = 0; // TODO get Real
      public static double kI = 0; // TODO get Real
      public static double kD = 0; // TODO get Real
      public static double minPIDOutput = -1.0;
      public static double maxPIDOutput = 1.0;

      public static double period = 0.02;

      // Encoder
      public static final int encoderChannelA = 5;
      public static final int encoderChannelB = 6;
      public static final boolean invertEncoder = false;
      public static final double gearRatio = 2;
      public static final double distancePerPulse = (2 * Math.PI) * gearRatio / 2048; // radians
      public static final Rotation2d cogOffset = Rotation2d.fromRadians(0); // TODO find this value

      // Positions
      public enum Position {
        DOWN(Rotation2d.fromDegrees(0)),
        CLIMB(Rotation2d.fromDegrees(0)),
        AMP(Rotation2d.fromDegrees(0));

        public final Rotation2d angle;

        private Position(Rotation2d angle) {
          this.angle = angle;
        }
      }

      // FF
      public static double kS = 0; // TODO GET REAL
      public static double kG = 0; // TODO get Real
      public static double kV = 0; // TODO get Real
      public static double kA = 0; // TODO get real

      // Encoder
      public static int portA = 0;
      public static int portB = 1;
      public static double pulsesPerRevolution = 2048;
      public static final double resetProfiledPIDControllerPos = 0;

      public static double maxVel = 0;
      public static double maxAccel = 0;

      public static double homeRad = 0; // TODO get real radians
    }

    public static class kFlywheels {
      public static boolean invert = true;
      public static double gearing = 4.0 / 3.0;
      public static double positionConversionFactor = gearing * 2 * Math.PI;
      public static double velocityConversionFactor = 1.0 / 60.0 * gearing * 2 * Math.PI;

      public static class kFlywheel1 {
        public static int canID = 13;
        public static double ks = 0;
        public static double kv = 0;
        public static double ka = 0;
        public static double minPIDOutput = 0;
        public static double maxPIDOutput = 0;
        public static double kP = 0;
        public static double kD = 0;
      }

      public static class kFlywheel2 {
        public static int canID = 14;
        public static double ks = 0;
        public static double kv = 0;
        public static double ka = 0;
        public static double minPIDOutput = 0;
        public static double maxPIDOutput = 0;
        public static double kP = 0;
        public static double kD = 0;
      }
    }

    public static class kHandoffRollers {
      public static int canID = 15;
      public static double shooterFeedVoltage = 7;
      public static double shooterFeedTime = 0.2;
      public static boolean inverted = false;
      public static int sensorChannel = 1; // TODO GET REAL ONE
    }
  }
}
