package frc.robot.utilities;

import static frc.robot.utilities.SparkConfigurator.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.kSwerve.kModule;
import java.util.Set;

public class MAXSwerve {
  private SwerveModuleState targetState = new SwerveModuleState();
  private final double chassisOffset;

  // Hardware
  private final CANSparkMax driveNEO;
  private final CANSparkMax steerNEO;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder steerEncoder;

  // Controls
  private final SparkPIDController drivePID;
  private final SparkPIDController steerPID;

  private final SimpleMotorFeedforward driveFF;

  // Logging
  private final NetworkTable moduleTable;
  private final DoublePublisher goalVelPub;
  private final DoublePublisher goalHeadingPub;
  private final DoublePublisher measVelPub;
  private final DoublePublisher measHeadingPub;
  private final DoubleArrayPublisher voltagesPub;

  // Simulation
  private double simDrivePosition = 0;

  public MAXSwerve(int driveCANId, int steerCANId, double offset, NetworkTable table) {
    chassisOffset = offset;

    // Initialize hardware
    driveNEO =
        getSparkMax(
            driveCANId,
            MotorType.kBrushless,
            false,
            Set.of(Sensors.INTEGRATED),
            Set.of(LogData.VOLTAGE, LogData.POSITION, LogData.VELOCITY));
    steerNEO =
        getSparkMax(
            steerCANId,
            MotorType.kBrushless,
            false,
            Set.of(Sensors.ABSOLUTE),
            Set.of(LogData.VOLTAGE, LogData.POSITION, LogData.VELOCITY));

    driveEncoder = driveNEO.getEncoder();
    steerEncoder = steerNEO.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    driveEncoder.setPositionConversionFactor(kModule.drivingEncoderPositionFactor);
    steerEncoder.setPositionConversionFactor(kModule.steeringEncoderPositionFactor);

    driveEncoder.setVelocityConversionFactor(kModule.drivingEncoderVelocityFactor);
    steerEncoder.setVelocityConversionFactor(kModule.steeringEncoderVelocityFactor);

    steerEncoder.setInverted(kModule.invertSteerEncoder);

    // Initialize controls objects
    drivePID = driveNEO.getPIDController();
    drivePID.setFeedbackDevice(driveEncoder);
    steerPID = steerNEO.getPIDController();
    steerPID.setFeedbackDevice(steerEncoder);

    driveFF = new SimpleMotorFeedforward(kModule.kDrive.kS, kModule.kDrive.kV, kModule.kDrive.kA);

    drivePID.setOutputRange(kModule.kDrive.minOutput, kModule.kDrive.maxOutput);
    steerPID.setOutputRange(kModule.kSteer.minOutput, kModule.kSteer.maxOutput);

    steerPID.setPositionPIDWrappingEnabled(true);
    steerPID.setPositionPIDWrappingMaxInput(kModule.steeringEncoderPositionPIDMaxInput);
    steerPID.setPositionPIDWrappingMinInput(kModule.steeringEncoderPositionPIDMinInput);

    drivePID.setP(kModule.kDrive.kP);
    drivePID.setD(kModule.kDrive.kD);

    steerPID.setP(kModule.kSteer.kP);
    steerPID.setD(kModule.kSteer.kD);

    // Configure motor controllers
    driveNEO.setIdleMode(IdleMode.kBrake);
    steerNEO.setIdleMode(IdleMode.kBrake);

    driveNEO.setSmartCurrentLimit(kModule.driveSmartCurrentLimit);
    driveNEO.setSecondaryCurrentLimit(kModule.driveMaxCurrent);
    steerNEO.setSmartCurrentLimit(kModule.steerSmartCurrentLimit);
    steerNEO.setSecondaryCurrentLimit(kModule.steerMaxCurrent);

    driveNEO.burnFlash();
    steerNEO.burnFlash();

    if (!RobotBase.isReal()) targetState.angle = new Rotation2d(steerEncoder.getPosition());

    // Start NT publishers for logging
    moduleTable = table;
    goalVelPub = moduleTable.getDoubleTopic("Goal Velocity").publish();
    goalHeadingPub = moduleTable.getDoubleTopic("Goal Heading").publish();
    measVelPub = moduleTable.getDoubleTopic("Measured Velocity").publish();
    measHeadingPub = moduleTable.getDoubleTopic("Measured Heading").publish();
    voltagesPub = moduleTable.getDoubleArrayTopic("Voltages").publish();
  }

  // Get the corrected (for chassis offset) heading
  public Rotation2d getCorrectedSteer() {
    if (RobotBase.isSimulation()) return targetState.angle;
    return new Rotation2d(steerEncoder.getPosition() + chassisOffset);
  }

  // Get the state of the module (vel, heading)
  public SwerveModuleState getState() {
    if (RobotBase.isSimulation()) return targetState;
    return new SwerveModuleState(driveEncoder.getVelocity(), getCorrectedSteer());
  }

  // Get the targeted state of the module (vel, heading)
  public SwerveModuleState getTargetState() {
    return targetState;
  }

  // Get the position of the module (wheel distance traveled, heading)
  public SwerveModulePosition getPositon() {
    if (RobotBase.isSimulation())
      return new SwerveModulePosition(simDrivePosition, getCorrectedSteer());
    return new SwerveModulePosition(driveEncoder.getPosition(), getCorrectedSteer());
  }

  // Get the error of the heading
  public Rotation2d getHeadingError() {
    return targetState.angle.minus(getCorrectedSteer());
  }

  // Set the module's target state
  public void setTargetState(SwerveModuleState desiredState, boolean closedLoopDrive) {
    // Optimize the state to prevent having to make a rotation of more than 90 degrees
    SwerveModuleState optimizedState =
        SwerveModuleState.optimize(
            new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle),
            getCorrectedSteer());

    // Scale
    optimizedState.speedMetersPerSecond *= Math.cos(Math.abs(getHeadingError().getRadians()));

    // Set the built-in PID for closed loop, or just give a regular voltage for open loop
    if (closedLoopDrive) {
      drivePID.setReference(
          optimizedState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          driveFF.calculate(optimizedState.speedMetersPerSecond));
    } else {
      driveNEO.setVoltage(driveFF.calculate(optimizedState.speedMetersPerSecond));
    }

    steerPID.setReference(
        optimizedState.angle.minus(new Rotation2d(chassisOffset)).getRadians(),
        ControlType.kPosition);

    // Record the target state
    targetState = optimizedState;
    // Forward euler on the position in sim
    if (RobotBase.isSimulation()) simDrivePosition += targetState.speedMetersPerSecond * 0.02;
  }

  // rawvolts output for SysId
  public void setRawDriveVoltage(double volts) {
    driveNEO.setVoltage(volts);
  }

  // gets the volts that are being applied
  public double getRawDriveNeoVoltage() {
    return driveNEO.getAppliedOutput() * driveNEO.getBusVoltage();
  }

  // Set the module to the chassis X configuraiton
  public void setX() {
    setTargetState(new SwerveModuleState(0, new Rotation2d(Math.PI / 4 + chassisOffset)), false);
  }

  // Sets motors all to look like an O from birdseye view, used for angular SysId
  public void setO() {
    setTargetState(
        new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4 + chassisOffset)), false);
  }

  // Reset the drive encoder to zero (reset for odometry)
  public void resetEncoder() {
    driveEncoder.setPosition(0);
    if (RobotBase.isSimulation()) simDrivePosition = 0;
  }

  // Put the drive motors into or out of brake mode
  public void setBrakeMode(boolean brake) {
    if (brake) {
      driveNEO.setIdleMode(IdleMode.kBrake);
    } else {
      driveNEO.setIdleMode(IdleMode.kCoast);
    }
  }

  // Update the logging values in NT
  public void updateNT() {
    goalVelPub.set(targetState.speedMetersPerSecond);
    goalHeadingPub.set(targetState.angle.getRadians());
    measVelPub.set(driveEncoder.getVelocity());
    measHeadingPub.set(MathUtil.angleModulus(getCorrectedSteer().getRadians()));
    voltagesPub.set(
        new double[] {
          driveNEO.getAppliedOutput() * driveNEO.getBusVoltage(),
          steerNEO.getAppliedOutput() * steerNEO.getBusVoltage()
        });
  }
}
