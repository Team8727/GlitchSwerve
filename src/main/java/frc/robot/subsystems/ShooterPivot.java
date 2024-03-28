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
import frc.robot.Constants.kShooter.kPivot.ShooterPosition;
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
  private ShooterPosition goalPosition = ShooterPosition.HOME;

  // Encoder objects
  private final Encoder pivotEncoder;
  private Rotation2d encoderOffset;

  private final TrapezoidProfile.State currentSetpoint;

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
        getFollower(
            pivotLeader, kPivot.pivotFollowerID, CANSparkLowLevel.MotorType.kBrushless, true);
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
    resetEncoder(ShooterPosition.HARDSTOPS.angle);

    // Controller Configs
    pivotController =
        new ProfiledPIDController(
            kPivot.kP,
            0,
            kPivot.kD,
            new TrapezoidProfile.Constraints(kPivot.maxVel, kPivot.maxAccel));
    goal = new TrapezoidProfile.State(ShooterPosition.HOME.angle.getRadians(), 0);

    pivotController.reset(getPivotAngle().getRadians());
    pivotController.setGoal(goal);
    currentSetpoint = pivotController.getSetpoint();

    this.setDefaultCommand(holdAngle());
    Shuffleboard.getTab("ShooterPivot").addString("Intake Position", () -> goalPosition.name());
  }

  // ---------- Commands ----------

  public Command goToPositionCommand(ShooterPosition position) {
    return this.runOnce(() -> goalPosition = position)
        .andThen(goToAngleCommand(position.angle))
        .withName("Go to position")
        .asProxy();
  }

  public Command goToAngleCommand(Rotation2d angle) {
    return this.runOnce(this::resetProfile)
        .andThen(setGoal(angle))
        .andThen(
            this.run(() -> pivotLeader.setVoltage(calculateVoltage(angle))).until(this::isAtGoal))
        .withName("Go to angle");
  }

  public Command holdAngle() {
    return this.run(
            () -> pivotLeader.setVoltage(calculateVoltage(Rotation2d.fromRadians(goal.position))))
        .withName("Hold angle");
  }

  public Command trackAngleCommand(Supplier<Rotation2d> angleSupplier) {
    return this.runOnce(this::resetProfile)
        .andThen(
            this.run(
                () -> {
                  goal.position = angleSupplier.get().getRadians();
                  goal.velocity = 0;
                  pivotController.setGoal(goal);
                  pivotLeader.setVoltage(calculateVoltage(angleSupplier.get()));
                }))
        .asProxy();
  }

  public Command setBrakeModeCommand(boolean on) {
    return this.runOnce(() -> setBrakeMode(on)).ignoringDisable(true);
  }

  public Command setEncoderHome() {
    return this.runOnce(() -> resetEncoder(ShooterPosition.HOME.angle)).ignoringDisable(true);
  }

  public Command setVoltage(DoubleSupplier voltageSupplier) {
    return this.run(() -> pivotLeader.setVoltage(voltageSupplier.getAsDouble())).asProxy();
  }

  public Command setGoal(Rotation2d angle) {
    return this.runOnce(
        () -> {
          goal.position = angle.getRadians();
          goal.velocity = 0;
          pivotController.setGoal(goal);
        });
  }

  // ---------- Public interface methods ----------

  public Command setVolts(double volts) {
    return this.run(() -> pivotLeader.setVoltage(volts)).finallyDo(() -> pivotLeader.setVoltage(0));
  }

  public void resetProfile() {
    pivotController.reset(getPivotAngle().getRadians(), getPivotVelocity());
  }

  @Log.NT
  public Rotation2d getPivotAngle() {
    return new Rotation2d(getRawEncoder().getRadians() + encoderOffset.getRadians());
  }

  @Log.NT
  public double getPivotVelocity() {
    return pivotEncoder.getRate();
  }

  @Log.NT
  public double getGoalAngle() {
    return goal.position;
  }

  @Log.NT
  public double getGoalVelocity() {
    return goal.velocity;
  }

  @Log.NT
  public double getSetpointPosition() {
    return currentSetpoint.position;
  }

  @Log.NT
  public double getSetpointVelocity() {
    return currentSetpoint.velocity;
  }

  @Log.NT
  public double getAppliedVoltage() {
    return pivotLeader.getAppliedOutput() * pivotLeader.getBusVoltage();
  }

  @Log.NT
  public boolean isHome() {
    return goalPosition == ShooterPosition.HOME && isAtGoal();
  }

  @Log.NT
  public boolean isAtGoal() {
    return Math.abs(getPivotAngle().getRadians() - getGoalAngle())
        < kPivot.atGoalDeadzone.getRadians();
  }

  @Log.NT
  public ShooterPosition getGoalPosition() {
    return goalPosition;
  }

  @Log.NT
  public String getRunningCommand() {
    var command = getCurrentCommand();
    if (command != null) return command.getName();
    else return "";
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
    // Get setpoint from profile
    var nextSetpoint = pivotController.getSetpoint();

    var accel = (nextSetpoint.velocity - currentSetpoint.velocity) / 0.02;

    // Calculate voltages
    double feedForwardVoltage =
        pivotFF.calculate(
            nextSetpoint.position + kPivot.cogOffset.getRadians(), nextSetpoint.velocity, accel);
    double feedbackVoltage = pivotController.calculate(getPivotAngle().getRadians());

    // Log Values
    this.log("FeedbackVoltage", feedbackVoltage);
    this.log("Feedforward voltage", feedForwardVoltage);

    currentSetpoint.position = nextSetpoint.position;
    currentSetpoint.velocity = nextSetpoint.velocity;

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
