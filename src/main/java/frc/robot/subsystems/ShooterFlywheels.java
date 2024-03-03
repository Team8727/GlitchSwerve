package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.kShooter.kFlywheels;
import frc.robot.Constants.kShooter.kFlywheels.kFlywheel1;
import frc.robot.Constants.kShooter.kFlywheels.kFlywheel2;
import frc.robot.commands.SysIdRoutines.SysIdType;
import frc.robot.utilities.SparkConfigurator.LogData;
import java.util.Set;
import monologue.Annotations.Log;

public class ShooterFlywheels extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax flywheel1;
  private final CANSparkMax flywheel2;

  // Controls Objects
  private final SimpleMotorFeedforward fly1FF;
  private final SimpleMotorFeedforward fly2FF;
  private final SparkPIDController fly1PID;
  private final SparkPIDController fly2PID;
  private final RelativeEncoder fly1Encoder;
  private final RelativeEncoder fly2Encoder;
  private double setpoint;

  public ShooterFlywheels() {
    flywheel1 =
        getSparkMax(
            kFlywheel1.canID,
            CANSparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(LogData.POSITION, LogData.VELOCITY, LogData.VOLTAGE));
    flywheel2 =
        getSparkMax(
            kFlywheel2.canID,
            CANSparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(LogData.POSITION, LogData.VELOCITY, LogData.VOLTAGE));

    flywheel1.setInverted(kFlywheels.invert);
    flywheel2.setInverted(!kFlywheels.invert);

    // FeedForwards
    fly1FF = new SimpleMotorFeedforward(kFlywheel1.ks, kFlywheel1.kv, kFlywheel1.ka);
    fly2FF = new SimpleMotorFeedforward(kFlywheel2.ks, kFlywheel2.kv, kFlywheel2.ka);

    // Encoders
    fly1Encoder = flywheel1.getEncoder();
    fly1Encoder.setPositionConversionFactor(kFlywheels.positionConversionFactor);
    fly1Encoder.setVelocityConversionFactor(kFlywheels.velocityConversionFactor);

    fly2Encoder = flywheel2.getEncoder();
    fly2Encoder.setPositionConversionFactor(kFlywheels.positionConversionFactor);
    fly2Encoder.setVelocityConversionFactor(kFlywheels.velocityConversionFactor);

    // PIDS
    fly1PID = flywheel1.getPIDController();
    fly1PID.setFeedbackDevice(fly1Encoder);
    fly1PID.setOutputRange(kFlywheel1.minPIDOutput, kFlywheel1.maxPIDOutput);
    fly1PID.setP(kFlywheel1.kP);
    fly1PID.setD(kFlywheel1.kD);

    fly2PID = flywheel2.getPIDController();
    fly2PID.setFeedbackDevice(fly2Encoder);
    fly2PID.setOutputRange(kFlywheel2.minPIDOutput, kFlywheel2.maxPIDOutput);
    fly2PID.setP(kFlywheel2.kP);
    fly2PID.setD(kFlywheel2.kD);
  }

  public Command setRollerSpeed(double vel) { // TODO make sure inverted correctly
    return this.run(() -> setVelocity(vel));
  }

  public Command shootTest(double voltage) {
    return this.startEnd(() -> setVoltage(voltage), () -> setVoltage(0));
  }

  // ---------- Public interface methods ----------

  @Log.NT
  public double[] getVelocities() {
    return new double[] {fly1Encoder.getVelocity(), fly2Encoder.getVelocity()};
  }

  @Log.NT
  public double getSetpointVelocity() {
    return setpoint;
  }

  @Log.NT
  public double[] getAppliedVoltages() {
    return new double[] {
      flywheel1.getAppliedOutput() * flywheel1.getBusVoltage(),
      flywheel2.getAppliedOutput() * flywheel2.getBusVoltage()
    };
  }

  public void setVoltage(double voltage) {
    flywheel1.setVoltage(voltage);
    flywheel2.setVoltage(voltage);
  }

  public void setVelocity(double velocity) {
    fly1PID.setReference(velocity, ControlType.kVelocity, 0, fly1FF.calculate(velocity));
    fly2PID.setReference(velocity, ControlType.kVelocity, 0, fly2FF.calculate(velocity));
  }

  public SysIdRoutine getRoutine(SysIdType type) {
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
              flywheel1.setVoltage(volts.magnitude());
              flywheel2.setVoltage(volts.magnitude());
            },
            (log) -> {
              fly1Encoder.setVelocityConversionFactor(Math.PI);
              fly2Encoder.setVelocityConversionFactor(Math.PI);

              log.motor("flywheel1Motor")
                  .voltage(appliedVoltage.mut_replace(flywheel1.getBusVoltage(), Volts))
                  .angularPosition(angle.mut_replace(fly1Encoder.getPosition() * Math.PI, Radians))
                  .angularVelocity(
                      velocity.mut_replace(
                          (fly1Encoder.getVelocity() * Math.PI), RadiansPerSecond));
              log.motor("flywheel2Motor")
                  .voltage(appliedVoltage.mut_replace(flywheel2.getBusVoltage(), Volts))
                  .angularPosition(angle.mut_replace(fly2Encoder.getPosition() * Math.PI, Radians))
                  .angularVelocity(
                      velocity.mut_replace(
                          (fly2Encoder.getVelocity() * Math.PI), RadiansPerSecond));
            },
            this));
  }

  public boolean getPieceCheck() {
    // return !pieceCheck.get(); // Invert because of sensor
    return false;
  }

  public void setHasPiece(boolean piece) {
    hasPiece = piece;
  }

  public boolean pieceState() {
    return hasPiece;
  }

  public Command intakeCommand() {
    return setRollerSpeed(kFlywheels.intakeVel)
        .until(() -> getPieceCheck())
        .finallyDo(() -> setHasPiece(true));
  }
}
