// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

  public enum TestMode {
    NONE,
    SYSID,
    NO_BRAKE_MODE
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

  public static class kSwerveShoot {
    public static Translation2d blueAmp =
        new Translation2d(1.8515, 8.2042); // Exact field-relative position
    public static Translation2d redAmp =
        new Translation2d(14.700758, 8.2042); // Exact field-relative position
    public static Rotation2d rotationAmpShot = new Rotation2d(-Math.PI / 2);

    public static Translation2d blueSpeaker = new Translation2d(-0.0381, 5.547868);
    public static Translation2d redSpeaker = new Translation2d(16.579342, 5.547868);

    // TODO tune this below:
    public static Translation2d chassisOffset =
        new Translation2d(0, 0.4); // Should be about half the distance of the chassis

    public static Translation2d stopDistance =
        new Translation2d(0, 0.1); // Should be about half the distance of the chassis

    public static final double spinupDistance = 4.5; // m
    public static final double ampPivotDistance = 2;
    public static final double autoAmpEnableRnage = 5; // m

    public static final double bumpUpSpeed = 0.9; // m/s
    public static final double bumpUpTime = 0.4; // s
  }

  public static class kClimber {
    public static int climberID = 18;

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

    public static final double rotationsToClimb = 90;
  }

  public static class kIntakeShooter {
    public static class kShootSpeaker {
      public static double shootVoltage = 9;
      public static double shootVelocity = 750;
      public static double delay = 1;
    }

    public static class kHandOff {
      public static double timeout = 0.75;
    }

    public static class kShootAmp {
      public static double shootVoltage = 5;
      public static double delay = 0.25;
    }
  }

  // Swerve subsystem constants (module constants included)
  public static class kSwerve {
    // Chassis dimensions from wheel center to center (meters)
    public static double width = Units.inchesToMeters(23);
    public static double length = width;

    // Speed & accel limits (m/s, rad/s, m/s^2 & rad/s^2)
    public static double maxTransSpeed = 5;
    public static double maxAngSpeed = 3 * Math.PI;

    public static double maxTransAccel = 1.35 * 9.81;
    public static double maxAngAccel = 10 * 2 * Math.PI;

    // Operator interface constants
    public static class Teleop {
      public static double translationGain = 1;
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

      public static final double transP = 12;

      public static final double maxOnTheFlyVel = 3;
      public static final double maxOnTheFlyAcc = 3;

      public static final HolonomicPathFollowerConfig pathFollowConfig =
          new HolonomicPathFollowerConfig(
              new PIDConstants(Auton.transP, 0.0, 0), // Translation PID constants
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
        public static final double kP = 0.25;
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
      public static final double wheelDiameter = 0.97 * Units.inchesToMeters(3);
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
            new Translation3d(-0.29972, -0.09552, 0.53),
            new Rotation3d(0, -Units.degreesToRadians(25), Math.PI));
    public static final Transform3d aprilTagCamera2PositionTransform =
        new Transform3d(
            new Translation3d(0.243, 0.193, 0.229),
            new Rotation3d(0, 0, 0)); // TODO GET REAL ONE this is from last year

    public static final Matrix<N3, N1> stateStdDevs =
        MatBuilder.fill(Nat.N3(), Nat.N1(), 0.02, 0.02, 0.01);
    public static final Matrix<N3, N1> visionStdDevs =
        MatBuilder.fill(Nat.N3(), Nat.N1(), 0.03, 0.03, 0.25);
    public static final double visionScalingFactor =
        2.3; // scaling factor applied to the visionStdDevs per meter bigger means trust less at a
    // distance
  }

  public static class kIntake {
    public static class kPivot {
      public static int pivotMotorID = 10;
      public static double period = 0.02;
      public static double minPIDOutput = -1.0;
      public static double maxPIDOutput = 1.0;
      // PID
      public static double kP = 1.5;
      public static double kI = 0;
      public static double kD = 0.03;
      // FF
      // public static double kS = 0.4273;
      public static double kS = 0.15;
      public static double kG = 0.18;
      public static double kV = 1.17;
      public static double kA = 0.043;

      // ProfiledPIDController
      public static double maxVel = 8;
      public static double maxAccel = 37;

      // Encoder
      public static int portA = 3;
      public static int portB = 2;
      public static double pulsesPerRevolution = 2048;
      public static final double encoderOffset = Math.PI; // TUNE THIS
      public static final double cogOffset = -0.4;
      public static boolean invertedEncoder = true;
      public static int gearRatio = 2;

      public static final double resetProfiledPIDControllerPos = 0;
      public static final double intakeRadiansDown = 0;
      public static final double intakeRadiansHome = Math.PI;

      public enum IntakePosition {
        HOME(Math.PI),
        DEPLOYED(0.05),
        EJECT(0.25 * Math.PI);
        public final double angle;

        private IntakePosition(double angle) {
          this.angle = angle;
        }
      }
    }

    public static class kRollers {
      public static int rollerMotorID = 9;
      public static boolean invert = true;
      public static int sensorChannel = 6;
      public static int currentLimit = 40;
      public static double rampRate = 0.08;

      // Intake tunable parameters ----------
      // Intake Command
      public static double intakeVoltage = 11;
      public static double softIntakeFromAmpVoltage = 3;
      public static double intakeTime = 0.25;
      public static double intakeDeployWait = 0.3;

      // Outtake tunable parameters
      public static double outtakeVoltage = -2.7;
      public static double ejectIntakeTime = 0.4;
    }
  }

  public static class kShooter {
    public static class kPivot {
      public static final int pivotLeaderID = 11;
      public static final int pivotFollowerID = 12;
      public static final boolean invertMotors = false;

      // PD
      public static double kP = 1.5;
      public static double kI = 0;
      public static double kD = 0;
      public static double minPIDOutput = -1.0;
      public static double maxPIDOutput = 1.0;

      public static double period = 0.02;

      // Encoder
      public static final int encoderChannelA = 0;
      public static final int encoderChannelB = 1;
      public static final boolean invertEncoder = true;
      public static final double gearRatio = 2;
      public static final double distancePerPulse = (2 * Math.PI) / gearRatio / 2048; // radians
      public static final Rotation2d cogOffset = Rotation2d.fromRadians(0.369);

      // Positions
      public enum ShooterPosition {
        HARDSTOPS(Rotation2d.fromDegrees(124)),
        HOME(Rotation2d.fromDegrees(124)),
        CLIMB(Rotation2d.fromDegrees(240)),
        AMP(Rotation2d.fromDegrees(200)),
        SOURCE(Rotation2d.fromDegrees(130)),
        ACTIVE_CONTROL;
        public final Rotation2d angle;

        private ShooterPosition(Rotation2d angle) {
          this.angle = angle;
        }

        private ShooterPosition() {
          this.angle = null;
        }
      }

      public static final Rotation2d atGoalDeadzone = Rotation2d.fromDegrees(10);

      // FF
      public static double kS = 0.16;
      public static double kG = -0.3;
      public static double kV = 1.4;
      public static double kA = 0.0;

      public static double maxVel = 6;
      public static double maxAccel = 15;
    }

    public static class kFlywheels {
      public static boolean invert = true;
      public static double gearing = 2.0 / 1.0;
      public static double positionConversionFactor = gearing * 2 * Math.PI; // rotations to radians
      public static double velocityConversionFactor =
          1.0 / 60.0 * gearing * 2 * Math.PI; // rpm to rad/s
      public static double shooterVelocityTolerance = 75; // rad/s

      public static class kFlywheel1 {
        public static int canID = 13;
        public static double ks = 0;
        public static double kv = 0.0152 * 0.66;
        public static double ka = 0;
        public static double minPIDOutput = -1;
        public static double maxPIDOutput = 1;
        public static double kP = 0.004;
        public static double kD = 0;
      }

      public static class kFlywheel2 {
        public static int canID = 14;
        public static double ks = 0;
        public static double kv = 0.0157 * 0.66;
        public static double ka = 0;
        public static double minPIDOutput = -1;
        public static double maxPIDOutput = 1;
        public static double kP = 0.004;
        public static double kD = 0;
      }
    }

    public static class kHandoffRollers {
      public static int canID = 15;
      public static double shooterFeedVoltage = 7;
      public static double sourceIntakeVoltage = -3;
      public static double intakeVoltage = 7;
      public static double shooterFeedTime = 0.2;
      public static double sourceIntakeFeedTime = 0.3;
      public static boolean inverted = false;

      // Sensors
      public static int upperSensorPort = 4;
      public static int lowerSensorPort = 5;
    }
  }

  public static class kIndications {
    public static final int ledPort = 0;
    public static final int leftStripLength = 23;
    public static final int rightStripLength = 23;
    public static final boolean invertDirection = false;
  }
}
