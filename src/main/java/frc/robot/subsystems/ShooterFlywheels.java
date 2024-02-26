package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.kShooter.kFlywheels;
import frc.robot.Constants.kShooter.kFlywheels.kFlywheel1;
import frc.robot.Constants.kShooter.kFlywheels.kFlywheel2;
import frc.robot.utilities.SparkConfigurator.LogData;
import frc.robot.utilities.SparkConfigurator.Sensors;
import java.util.Set;

public class ShooterFlywheels extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax flywheel1;
  private final CANSparkMax flywheel2;

  // Encoders
  private final AbsoluteEncoder fly1Encoder;
  private final AbsoluteEncoder fly2Encoder;

  // Feedforwards
  private final SimpleMotorFeedforward fly1FF;
  private final SimpleMotorFeedforward fly2FF;

  // PID Controllers
  private final SparkPIDController fly1PID;
  private final SparkPIDController fly2PID;

  // Sensor
  private DigitalInput pieceCheck;

  private boolean hasPiece = false;

  // SysId
  private final SysIdRoutine angularRoutine;
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe angular distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Radians.of(0));
  // Mutable holder for unit-safe angular velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RadiansPerSecond.of(0));

  public ShooterFlywheels() {
    flywheel1 =
        getSparkMax(
            kFlywheel1.canID,
            CANSparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(Sensors.ABSOLUTE),
            Set.of(LogData.POSITION, LogData.VELOCITY, LogData.VOLTAGE));
    flywheel2 =
        getSparkMax(
            kFlywheel2.canID,
            CANSparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(Sensors.ABSOLUTE),
            Set.of(LogData.POSITION, LogData.VELOCITY, LogData.VOLTAGE));

    // FeedForwards
    fly1FF = new SimpleMotorFeedforward(kFlywheel1.ks, kFlywheel1.kv, kFlywheel1.ka);
    fly2FF = new SimpleMotorFeedforward(kFlywheel2.ks, kFlywheel2.kv, kFlywheel2.ka);

    // Encoders
    fly1Encoder = flywheel1.getAbsoluteEncoder(Type.kDutyCycle);
    fly1Encoder.setPositionConversionFactor(kFlywheels.encoderPositionFactor);
    fly1Encoder.setInverted(false);

    fly2Encoder = flywheel2.getAbsoluteEncoder(Type.kDutyCycle);
    fly2Encoder.setPositionConversionFactor(kFlywheels.encoderPositionFactor);
    fly2Encoder.setInverted(false);

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

    // SysId
    angularRoutine =
        new SysIdRoutine(
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
                      .voltage(m_appliedVoltage.mut_replace(flywheel1.getBusVoltage(), Volts))
                      .angularPosition(
                          m_angle.mut_replace(fly1Encoder.getPosition() * Math.PI, Radians))
                      .angularVelocity(
                          m_velocity.mut_replace(
                              (fly1Encoder.getVelocity() * Math.PI), RadiansPerSecond));
                  log.motor("flywheel2Motor")
                      .voltage(m_appliedVoltage.mut_replace(flywheel2.getBusVoltage(), Volts))
                      .angularPosition(
                          m_angle.mut_replace(fly2Encoder.getPosition() * Math.PI, Radians))
                      .angularVelocity(
                          m_velocity.mut_replace(
                              (fly2Encoder.getVelocity() * Math.PI), RadiansPerSecond));
                },
                this));
    pieceCheck = new DigitalInput(kFlywheels.sensorChannel);
  }

  public Command setRollerSpeed(double vel) { // TODO make sure inverted correctly
    return this.run(
        () -> {
          fly1PID.setReference(vel, ControlType.kVelocity, 0, fly1FF.calculate(vel));
          fly2PID.setReference(-vel, ControlType.kVelocity, 0, fly2FF.calculate(-vel));
        });
  }

  public SysIdRoutine getAngularRoutine() {
    return angularRoutine;
  }

  public boolean getPieceCheck() {
    return !pieceCheck.get(); // Invert because of sensor
  }

  public void setHasPiece(boolean piece) {
    hasPiece = piece;
  }

  public Command intakeCommand() {
    return setRollerSpeed(kFlywheels.intakeVel)
        .until(() -> getPieceCheck())
        .finallyDo(() -> setHasPiece(true));
  }
}
