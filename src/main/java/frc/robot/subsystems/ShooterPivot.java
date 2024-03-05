package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utilities.SparkConfigurator.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.kShooter.kPivot;
import frc.robot.Constants.kShooter.kPivot.Position;
import frc.robot.commands.SysIdRoutines.SysIdType;
import frc.robot.utilities.Characterizable;
import frc.robot.utilities.SparkConfigurator.LogData;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class ShooterPivot extends SubsystemBase implements Logged, Characterizable {
  // Motorcontrollers
  private final CANSparkMax pivotLeader;
  private final CANSparkMax pivotFollower;

  // Controls objects
  private final ArmFeedforward pivotFF;
  private final ProfiledPIDController pivotController;
  private final TrapezoidProfile.State goal;
  private Position goalPosition = Position.HOME;

  // Encoder objects
  private final Encoder pivotEncoder;
  private Rotation2d encoderOffset;

  public ShooterPivot() {
    // Motor Initializations
    pivotLeader =
        getSparkMax(
            kPivot.pivotLeaderID,
            CANSparkLowLevel.MotorType.kBrushless,
            true,
            Set.of(),
            Set.of(LogData.POSITION, LogData.VELOCITY, LogData.VOLTAGE));
    pivotFollower =
        getFollower(pivotLeader, kPivot.pivotFollowerID, CANSparkLowLevel.MotorType.kBrushless);
    pivotLeader.setInverted(kPivot.invertMotors);
    pivotFollower.setInverted(!kPivot.invertMotors);
    setBrakeMode(true);

    pivotLeader.burnFlash();
    pivotFollower.burnFlash();

    // Feed Forwards
    pivotFF = new ArmFeedforward(kPivot.kS, kPivot.kG, kPivot.kV, kPivot.kA);

    // Encoder Configs
    pivotEncoder =
        new Encoder(kPivot.encoderChannelA, kPivot.encoderChannelB, kPivot.invertEncoder);
    pivotEncoder.setDistancePerPulse(kPivot.distancePerPulse);
    resetEncoder(Position.HOME.angle);

    // Controller Configs
    pivotController =
        new ProfiledPIDController(
            kPivot.kP,
            0,
            kPivot.kD,
            new TrapezoidProfile.Constraints(kPivot.maxVel, kPivot.maxAccel));
    goal = new TrapezoidProfile.State(getPivotAngle().getRadians(), getPivotVelocity());
    pivotController.reset(goal);
    pivotController.setGoal(goal);
  }

  // ---------- Commands ----------

  public Command goToPositionCommand(Position position) {
    return this.runOnce(() -> goalPosition = position).andThen(goToAngleCommand(position.angle));
  }

  public Command goToAngleCommand(Rotation2d angle) {
    return this.runOnce(this::resetProfile)
        .andThen(this.run(() -> pivotLeader.setVoltage(calculateVoltage(angle))));
  }

  public Command trackAngleCommand(Supplier<Rotation2d> angleSupplier) {
    return this.runOnce(this::resetProfile)
        .andThen(this.run(() -> pivotLeader.setVoltage(calculateVoltage(angleSupplier.get()))));
  }

  public Command setBrakeModeCommand(boolean on) {
    return this.runOnce(() -> setBrakeMode(on)).ignoringDisable(true);
  }

  public Command setEncoderHome() {
    return this.runOnce(() -> resetEncoder(Position.HOME.angle)).ignoringDisable(true);
  }

  public Command setVoltage(DoubleSupplier voltageSupplier) {
    return this.run(() -> pivotLeader.setVoltage(voltageSupplier.getAsDouble()));
  }

  public Command testCommand() {
    var entry = Shuffleboard.getTab("ShooterPivot").add("Voltage", 0).getEntry();
    return setVoltage(() -> entry.getDouble(0));
  }

  // ---------- Public interface methods ----------

  public void resetProfile() {
    pivotController.reset(getPivotAngle().getRadians(), getPivotVelocity());
  }

  @Log.NT
  public Rotation2d getPivotAngle() {
    return getRawEncoder().plus(encoderOffset);
  }

  @Log.NT
  public double getPivotVelocity() {
    return pivotEncoder.getRate();
  }

  @Log.NT
  public double getSetpointPosition() {
    return goal.position;
  }

  @Log.NT
  public double getSetpointVelocity() {
    return goal.velocity;
  }

  @Log.NT
  public double getAppliedVoltage() {
    return pivotLeader.getAppliedOutput() * pivotLeader.getBusVoltage();
  }

  @Log.NT
  public boolean isHome() {
    return goalPosition == Position.HOME && isAtGoal();
  }

  public boolean isAtGoal() {
    return Math.abs(getPivotAngle().getDegrees() - getSetpointPosition()) < kPivot.atGoalDeadzone;
  }

  public void setBrakeMode(boolean on) {
    if (on) {
      pivotLeader.setIdleMode(IdleMode.kBrake);
      pivotFollower.setIdleMode(IdleMode.kBrake);
    } else {
      pivotLeader.setIdleMode(IdleMode.kCoast);
      pivotFollower.setIdleMode(IdleMode.kCoast);
    }
  }

  public double calculateVoltage(Rotation2d angle) {
    // Set appropriate goal
    goal.position = angle.getRadians();
    goal.velocity = 0;
    pivotController.setGoal(goal);

    // Get setpoint from profile
    var profileSetpoint = pivotController.getSetpoint();

    // Calculate voltages
    double feedForwardVoltage =
        pivotFF.calculate(
            profileSetpoint.position + kPivot.cogOffset.getRadians(), profileSetpoint.velocity);
    double feedbackVoltage = pivotController.calculate(getPivotAngle().getRadians());

    return feedForwardVoltage + feedbackVoltage;
  }

  // ---------- Private hardware interface methods ----------

  private Rotation2d getRawEncoder() {
    return Rotation2d.fromRadians(pivotEncoder.getDistance());
  }

  private void resetEncoder(Rotation2d angle) {
    encoderOffset = angle.minus(getRawEncoder());
  }

  // Get SysID Routine
  public SysIdRoutine getRoutine(SysIdType type) {
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe angular distance values, persisted to avoid reallocation.
    MutableMeasure<Angle> angle = mutable(Radians.of(0));
    // Mutable holder for unit-safe angular velocity values, persisted to avoid reallocation.
    MutableMeasure<Velocity<Angle>> velocity = mutable(RadiansPerSecond.of(0));
    MutableMeasure<Voltage> stepVoltage = mutable(Volts.of(4));
    MutableMeasure<Time> timeout = mutable(Seconds.of(5));
    return new SysIdRoutine(
        new SysIdRoutine.Config(null, stepVoltage, timeout),
        new SysIdRoutine.Mechanism(
            (volts) -> {
              pivotLeader.setVoltage(volts.magnitude());
            },
            (log) -> {
              log.motor("shooterPivotMotor")
                  .voltage(appliedVoltage.mut_replace(getAppliedVoltage(), Volts))
                  .angularPosition(angle.mut_replace(getPivotAngle().getRadians(), Radians))
                  .angularVelocity(velocity.mut_replace(getPivotVelocity(), RadiansPerSecond));
            },
            this));
  }

  // Logging
  @Log.NT
  public double getEncoderPos() {
    return pivotEncoder.getDistance();
  }

  @Log.NT
  public double getEncoderVel() {
    return pivotEncoder.getRate();
  }

  @Log.NT
  public double getAppliedVolts() {
    return pivotLeader.getBusVoltage() * pivotLeader.getAppliedOutput();
  }
}
