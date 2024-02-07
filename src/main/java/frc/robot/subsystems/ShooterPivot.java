package frc.robot.subsystems;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kShooter.kPivot;

public class ShooterPivot extends SubsystemBase {
  // Motorcontrollers
  private CANSparkMax pivotMotor1;
  private CANSparkMax pivotMotor2;

  // PID controllers
  private final SparkPIDController pivot1PID;
  private final SparkPIDController pivot2PID;

  // Absolute Encoders
  private final AbsoluteEncoder pivot1Encoder;
  private final AbsoluteEncoder pivot2Encoder;

  public ShooterPivot() {
    // Motor Initializations
    pivotMotor1 = getSparkMax(kPivot.kMotor1.pivotMotorID, CANSparkLowLevel.MotorType.kBrushless);
    pivotMotor2 = getSparkMax(kPivot.kMotor2.pivotMotorID, CANSparkLowLevel.MotorType.kBrushless);

    // Encoder Configs
    pivot1Encoder = pivotMotor1.getAbsoluteEncoder(Type.kDutyCycle);
    pivot1Encoder.setPositionConversionFactor(kPivot.intakePivotEncoderPositionFactor);
    pivot1Encoder.setInverted(false);

    pivot2Encoder = pivotMotor2.getAbsoluteEncoder(Type.kDutyCycle);
    pivot2Encoder.setPositionConversionFactor(kPivot.intakePivotEncoderPositionFactor);
    pivot2Encoder.setInverted(false);

    // PID Configs
    pivot1PID = pivotMotor1.getPIDController();
    pivot1PID.setOutputRange(kPivot.minPIDOutput, kPivot.maxPIDOutput);
    pivot1PID.setPositionPIDWrappingEnabled(false);
    pivot1PID.setP(kPivot.kMotor1.kP);
    pivot1PID.setD(kPivot.kMotor1.kD);

    pivot2PID = pivotMotor2.getPIDController();
    pivot2PID.setOutputRange(kPivot.minPIDOutput, kPivot.maxPIDOutput);
    pivot2PID.setPositionPIDWrappingEnabled(false);
    pivot2PID.setP(kPivot.kMotor2.kP);
    pivot2PID.setD(kPivot.kMotor2.kD);
  }

  // Set position input radians
  public Command setIntakePivotPos(double posRad) {
    return this.run(
        () -> {
          pivot1PID.setReference(posRad, ControlType.kPosition);
          pivot2PID.setReference(posRad, ControlType.kPosition);
        });
  }
}
