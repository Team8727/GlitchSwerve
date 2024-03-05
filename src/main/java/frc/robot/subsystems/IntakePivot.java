package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utilities.SparkConfigurator.*;
import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.kIntake.kPivot;
import frc.robot.commands.SysIdRoutines.SysIdType;
import frc.robot.utilities.Characterizable;
import frc.robot.utilities.SparkConfigurator.Sensors;
import java.util.Set;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class IntakePivot extends SubsystemBase implements Characterizable, Logged {

  private final CANSparkMax pivotMotor;
  private final Encoder pivotEncoder;
  private double encoderOffset;

  // Controls
  private final ArmFeedforward pivotFF;
  private final ProfiledPIDController profiledPIDController;
  private final TrapezoidProfile.Constraints constraints =
      new Constraints(kPivot.maxVel, kPivot.maxAccel);

  // Shuffleboard
  private ShuffleboardTab tab = Shuffleboard.getTab("Active Configs");

  public IntakePivot() {
    // Motor Configs
    pivotMotor =
        getSparkMax(
            kPivot.pivotMotorID,
            CANSparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(Sensors.ABSOLUTE),
            Set.of(LogData.POSITION, LogData.VELOCITY, LogData.VOLTAGE));
    pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

    // Encoder Configs
    pivotEncoder = new Encoder(kPivot.portA, kPivot.portB);
    pivotEncoder.setReverseDirection(kPivot.invertedEncoder);
    resetEncoder();
    resetEncoderOffset(kPivot.intakeRadiansHome);

    // Feedforward Configs
    pivotFF = new ArmFeedforward(kPivot.kS, kPivot.kG, kPivot.kV, kPivot.kA);

    pivotEncoder.setDistancePerPulse(2 * Math.PI / (kPivot.pulsesPerRevolution * kPivot.gearRatio));
    pivotEncoder.reset();

    profiledPIDController = new ProfiledPIDController(kPivot.kP, kPivot.kI, kPivot.kD, constraints);
    profiledPIDController.reset(getPhysAngle());
    profiledPIDController.disableContinuousInput();

    // Button to Reset Encoder
    tab.add("Reset Intake Pivot Encoder", resetEncoder());
    tab.add("PID", profiledPIDController);
  }

  @Log.NT
  public double getPhysAngle() {
    return Math.PI - pivotEncoder.getDistance();
  }

  // MAIN CONTROLS -------------------------------
  public Command setIntakeDown(boolean setDown) {
    return setDown
        ? setIntakePivotPos(kPivot.intakeRadiansDown)
        : setIntakePivotPos(kPivot.intakeRadiansHome);
  }

  public Command setIntakePivotPos(double posRad) {
    return this.runOnce(this::resetProfile)
        .andThen(
            this.run(
                    () -> {
                      pivotMotor.setVoltage(calculateVoltage(posRad));
                    })
                .finallyDo(() -> pivotMotor.setVoltage(0)));
  }

  public Command setVoltageTest(DoubleSupplier volts) {
    return this.startEnd(
        () -> pivotMotor.setVoltage(volts.getAsDouble()), () -> pivotMotor.setVoltage(0));
  }

  // ---------- Public interface methods ----------

  public void resetProfile() {
    profiledPIDController.reset(getPhysAngle(), getPivotVelocity());
  }

  @Log.NT
  public double getPivotVelocity() {
    return pivotEncoder.getRate();
  }

  @Log.NT
  public double getAppliedVoltage() {
    return pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
  }

  public void setBrakeMode(boolean on) {
    if (on) {
      pivotMotor.setIdleMode(IdleMode.kBrake);
    } else {
      pivotMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public double calculateVoltage(double angle) {
    // Set appropriate goal
    profiledPIDController.setGoal(angle);

    // Get setpoint from profile
    var profileSetpoint = profiledPIDController.getSetpoint();

    // Calculate voltages
    double feedForwardVoltage =
        pivotFF.calculate(profileSetpoint.position, profileSetpoint.velocity);
    double feedbackVoltage = profiledPIDController.calculate(getPhysAngle());

    // Log Values
    this.log("FeedbackVoltage", feedbackVoltage);
    this.log("FeedForwardPosition", profileSetpoint.position);
    this.log("FeedForwardVelocity", profileSetpoint.velocity);

    return feedForwardVoltage + feedbackVoltage;
  }

  public boolean isHome() {
    return ((profiledPIDController.getGoal().position - getPhysAngle()) < 0.1)
        && (profiledPIDController.getGoal().position == kPivot.intakeRadiansHome);
  }

  // Private hardware
  private double getRawEncoder() {
    return pivotEncoder.getDistance();
  }

  private void resetEncoderOffset(double angle) {
    encoderOffset = angle - getRawEncoder();
  }

  // Reset Encoder
  public Command resetEncoder() {
    return this.runOnce(() -> pivotEncoder.reset()).ignoringDisable(true);
  }

  // Return SysId Routine
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
              pivotMotor.setVoltage(volts.magnitude());
            },
            (log) -> {
              log.motor("intakePivotMotor")
                  .voltage(
                      appliedVoltage.mut_replace(
                          pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput(), Volts))
                  .angularPosition(angle.mut_replace(pivotEncoder.getDistance() * Math.PI, Radians))
                  .angularVelocity(velocity.mut_replace(pivotEncoder.getRate(), RadiansPerSecond));
            },
            this));
  }
}
