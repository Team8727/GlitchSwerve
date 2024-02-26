package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utilities.SparkConfigurator.*;

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
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.kShooter.kPivot;
import frc.robot.utilities.SparkConfigurator.LogData;
import frc.robot.utilities.SparkConfigurator.Sensors;
import java.util.Set;
import monologue.Annotations.Log;
import monologue.Logged;

public class ShooterPivot extends SubsystemBase implements Logged {
  // Motorcontrollers
  private final CANSparkMax pivotMotor1;
  private final CANSparkMax pivotMotor2;

  // Feedforwards
  private final ArmFeedforward pivotFF;

  // Absolute Encoders
  private final AbsoluteEncoder pivotEncoder;

  // Profile Stuff
  private final TrapezoidProfile.Constraints constraints =
      new Constraints(kPivot.maxVel, kPivot.maxAccel);
  private final ProfiledPIDController profiledPIDController;

  // SysId Objects
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe angular distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Radians.of(0));
  // Mutable holder for unit-safe angular velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RadiansPerSecond.of(0));

  private final SysIdRoutine angularRoutine;

  public ShooterPivot() {
    // Motor Initializations
    pivotMotor1 =
        getSparkMax(
            kPivot.pivot1MotorID,
            CANSparkLowLevel.MotorType.kBrushless,
            true,
            Set.of(Sensors.ABSOLUTE),
            Set.of(LogData.POSITION, LogData.VELOCITY, LogData.VOLTAGE));
    pivotMotor2 =
        getFollower(pivotMotor1, kPivot.pivot2MotorID, CANSparkLowLevel.MotorType.kBrushless);
    pivotMotor1.setIdleMode(CANSparkBase.IdleMode.kBrake);
    pivotMotor2.setIdleMode(CANSparkBase.IdleMode.kBrake);

    // Feed Forwards
    pivotFF = new ArmFeedforward(kPivot.kS, kPivot.kG, kPivot.kV, kPivot.kA);

    // Encoder Configs
    pivotEncoder = pivotMotor1.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(kPivot.shooterPivotEncoderPositionFactor);
    pivotEncoder.setVelocityConversionFactor(kPivot.shooterPivotEncoderVelocityFactor);
    pivotEncoder.setInverted(false);

    // ProfiledPIDController
    profiledPIDController = new ProfiledPIDController(kPivot.kP, kPivot.kI, kPivot.kD, constraints);

    // SysId object
    angularRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                  pivotMotor1.setVoltage(volts.magnitude());
                },
                (log) -> {
                  pivotEncoder.setVelocityConversionFactor(Math.PI);
                  log.motor("shooterPivotMotor")
                      .voltage(m_appliedVoltage.mut_replace(pivotMotor1.getBusVoltage(), Volts))
                      .angularPosition(
                          m_angle.mut_replace(pivotEncoder.getPosition() * Math.PI, Radians))
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
              pivotMotor1.setVoltage(pivotFF.calculate(posRad, setpoint) + setpoint);
            })
        .until(() -> profiledPIDController.atGoal());
  }

  public Command setShooterHome() {
    return setIntakePivotPos(kPivot.homeRad);
  }

  // Get SysID Routine
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
    return pivotMotor1.getBusVoltage() * pivotMotor1.getAppliedOutput();
  }
}
