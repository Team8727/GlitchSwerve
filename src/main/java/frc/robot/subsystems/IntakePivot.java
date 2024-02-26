package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utilities.SparkConfigurator.*;
import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.kIntake.kPivot;
import frc.robot.utilities.SparkConfigurator.LogData;
import frc.robot.utilities.SparkConfigurator.Sensors;
import java.util.Set;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class IntakePivot extends SubsystemBase implements Logged {

  private final CANSparkMax pivotMotor;
  private final AbsoluteEncoder pivotEncoder;

  private final ArmFeedforward pivotFF;

  private final SysIdRoutine angularRoutine;

  // Profile Stuff
  private final ProfiledPIDController profiledPIDController;
  private final TrapezoidProfile.Constraints constraints =
      new Constraints(kPivot.maxVel, kPivot.maxAccel);

  // SysId Objects
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe angular distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Radians.of(0));
  // Mutable holder for unit-safe angular velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RadiansPerSecond.of(0));
  private final MutableMeasure<Voltage> stepVoltage = mutable(Volts.of(4));
  private final MutableMeasure<Time> timout = mutable(Seconds.of(5));

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
    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(kPivot.intakePivotEncoderPositionFactor);
    pivotEncoder.setVelocityConversionFactor(kPivot.intakePivotEncoderVelocityFactor);
    pivotEncoder.setInverted(false);

    // Feedforward Configs
    pivotFF = new ArmFeedforward(kPivot.kS, kPivot.kG, kPivot.kV, kPivot.kA);

    profiledPIDController = new ProfiledPIDController(kPivot.kP, kPivot.kI, kPivot.kD, constraints);
    profiledPIDController.reset(pivotEncoder.getPosition());

    // Angular Routine Configs
    angularRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(null, stepVoltage, timout),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                  pivotMotor.setVoltage(volts.magnitude());
                },
                (log) -> {
                  log.motor("intakePivotMotor")
                      .voltage(
                          m_appliedVoltage.mut_replace(
                              pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput(), Volts))
                      .angularPosition(m_angle.mut_replace(pivotEncoder.getPosition(), Radians))
                      .angularVelocity(
                          m_velocity.mut_replace(
                              (pivotEncoder.getVelocity() * Math.PI), RadiansPerSecond));
                },
                this));
  }

  // Set position input radians
  public Command setIntakePivotPos(double posRad) {
    return this.run(
            () -> {
              var setpoint = profiledPIDController.calculate(pivotEncoder.getPosition(), posRad);
              pivotMotor.setVoltage(pivotFF.calculate(posRad, setpoint) + setpoint);
            })
        .until(() -> profiledPIDController.atGoal());
  }

  public Command setIntakeDown(boolean setDown) {
    return setDown
        ? setIntakePivotPos(kPivot.intakeRadiansDown)
        : setIntakePivotPos(kPivot.intakeRadiansHome);
  }

  public Command setVoltageTest(DoubleSupplier volts) {
    return this.startEnd(
        () -> pivotMotor.setVoltage(volts.getAsDouble()), () -> pivotMotor.setVoltage(0));
  }

  public SysIdRoutine getAngularRoutine() {
    return angularRoutine;
  }

  // Logging
  @Log.NT
  public double getEncoderPos() {
    return pivotEncoder.getPosition();
  }

  @Log.NT
  public double getEncoderVel() {
    return pivotEncoder.getVelocity();
  }

  @Log.NT
  public double getAppliedVolts() {
    return pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput();
  }
}
