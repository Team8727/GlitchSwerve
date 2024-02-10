package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utilities.SparkConfigurator.*;
import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.kIntake.kPivot;
import frc.robot.utilities.SparkConfigurator.Sensors;
import java.util.Set;

public class IntakePivot extends SubsystemBase {

  private final CANSparkMax pivotMotor;
  private final AbsoluteEncoder pivotEncoder;

  private final ArmFeedforward pivotFF;
  private final SparkPIDController pivotPID;

  private final SysIdRoutine angularRoutine;

  // Profile Stuff
  private final TrapezoidProfile.Constraints constraints =
      new Constraints(kPivot.kProfile.maxVel, kPivot.kProfile.maxAccel);
  private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  // SysId Objects
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe angular distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Radians.of(0));
  // Mutable holder for unit-safe angular velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RadiansPerSecond.of(0));

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
    pivotEncoder.setInverted(false);

    // PID Configs
    pivotPID = pivotMotor.getPIDController();
    pivotPID.setFeedbackDevice(pivotEncoder);
    pivotPID.setOutputRange(kPivot.minPIDOutput, kPivot.maxPIDOutput);
    pivotPID.setP(kPivot.kP);
    pivotPID.setD(kPivot.kD);

    // Feedforward Configs
    pivotFF = new ArmFeedforward(kPivot.kS, kPivot.kG, kPivot.kV, kPivot.kA);

    // Angular Routine Configs
    angularRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                  pivotMotor.setVoltage(volts.magnitude());
                },
                (log) -> {
                  pivotEncoder.setVelocityConversionFactor(Math.PI);
                  log.motor("intakePivotMotor")
                      .voltage(m_appliedVoltage.mut_replace(pivotMotor.getBusVoltage(), Volts))
                      .angularPosition(
                          m_angle.mut_replace(pivotEncoder.getPosition() * Math.PI, Radians))
                      .angularVelocity(
                          m_velocity.mut_replace(
                              (pivotEncoder.getVelocity() * Math.PI), RadiansPerSecond));
                },
                this));
  }

  // Acts as a filter calculating setpoints for the PID control
  private TrapezoidProfile.State calculateSetpoint(double posRad) {
    goal = new TrapezoidProfile.State(posRad, 0);
    setpoint = profile.calculate(kPivot.period, setpoint, goal);
    return setpoint;
  }

  private boolean getDone() {
    return goal.position == pivotEncoder.getPosition()
        && goal.velocity == pivotEncoder.getVelocity();
  }

  // Set position input radians
  public Command setIntakePivotPos(double posRad) {
    return this.run(
            () -> {
              TrapezoidProfile.State pos = calculateSetpoint(posRad);
              pivotPID.setReference(
                  pos.position,
                  ControlType.kPosition,
                  0,
                  pivotFF.calculate(pos.position, pos.velocity));
            })
        .until(this::getDone);
  }

  public SysIdRoutine getAngularRoutine() {
    return angularRoutine;
  }
}
