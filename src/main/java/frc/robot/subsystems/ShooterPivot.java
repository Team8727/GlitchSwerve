package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utilities.SparkConfigurator.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
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
  private final Encoder pivotEncoder;

  // Profile Stuff
  private final TrapezoidProfile.Constraints constraints =
      new Constraints(kPivot.maxVel, kPivot.maxAccel);
  private final ProfiledPIDController profiledPIDController;

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
    pivotEncoder = new Encoder(kPivot.portA, kPivot.portB);
    pivotEncoder.setDistancePerPulse(2 * Math.PI / kPivot.pulsesPerRevolution);

    // PID Configs
    pivotPID = pivotMotor1.getPIDController();
    pivotPID.setFeedbackDevice(pivotEncoder);
    pivotPID.setOutputRange(kPivot.minPIDOutput, kPivot.maxPIDOutput);
    pivotPID.setPositionPIDWrappingEnabled(false);
    pivotPID.setP(kPivot.kP);
    pivotPID.setD(kPivot.kD);
  }

  // Set position input radians
  public Command setIntakePivotPos(double posRad) {
    return this.run(
            () -> {
              var setpoint = profiledPIDController.calculate(pivotEncoder.getDistance(), posRad);
              pivotMotor1.setVoltage(pivotFF.calculate(posRad, setpoint) + setpoint);
            })
        .until(() -> profiledPIDController.atGoal());
  }

  public Command setShooterHome() {
    return setIntakePivotPos(kPivot.homeRad);
  }

  // Get SysID Routine
  public SysIdRoutine getAngularRoutine() {
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe angular distance values, persisted to avoid reallocation.
    MutableMeasure<Angle> angle = mutable(Radians.of(0));
    // Mutable holder for unit-safe angular velocity values, persisted to avoid reallocation.
    MutableMeasure<Velocity<Angle>> velocity = mutable(RadiansPerSecond.of(0));
    return new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (volts) -> {
              pivotMotor1.setVoltage(volts.magnitude());
            },
            (log) -> {
              pivotEncoder.setVelocityConversionFactor(Math.PI);
              log.motor("shooterPivotMotor")
                  .voltage(appliedVoltage.mut_replace(pivotMotor1.getBusVoltage(), Volts))
                  .angularPosition(angle.mut_replace(pivotEncoder.getPosition() * Math.PI, Radians))
                  .angularVelocity(
                      velocity.mut_replace(
                          (pivotEncoder.getVelocity() * Math.PI), RadiansPerSecond));
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
    return pivotMotor1.getBusVoltage() * pivotMotor1.getAppliedOutput();
  }
}
