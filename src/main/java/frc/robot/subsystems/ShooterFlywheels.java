package frc.robot.subsystems;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kShooter.kFlywheels;
import frc.robot.Constants.kShooter.kFlywheels.kFlywheel1;
import frc.robot.Constants.kShooter.kFlywheels.kFlywheel2;

public class ShooterFlywheels extends SubsystemBase {

  private CANSparkMax flywheel1;
  private CANSparkMax flywheel2;

  private AbsoluteEncoder fly1Encoder;
  private AbsoluteEncoder fly2Encoder;

  private SimpleMotorFeedforward fly1FF;
  private SimpleMotorFeedforward fly2FF;

  private SparkPIDController fly1PID;
  private SparkPIDController fly2PID;

  public ShooterFlywheels() {
    flywheel1 = getSparkMax(kFlywheel1.canID, CANSparkLowLevel.MotorType.kBrushless);
    flywheel2 = getSparkMax(kFlywheel2.canID, CANSparkLowLevel.MotorType.kBrushless);

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
  }

  public Command setRollerSpeed(double vel) { // TODO make sure inverted correctly
    return this.run(
        () -> {
          fly1PID.setReference(vel, ControlType.kVelocity, 0, fly1FF.calculate(vel));
          fly2PID.setReference(-vel, ControlType.kVelocity, 0, fly2FF.calculate(-vel));
        });
  }
}
