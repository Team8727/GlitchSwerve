package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimber;
import java.util.function.DoubleSupplier;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

public class Climber extends SubsystemBase {

  CANSparkMax climbMotor;
  SimpleMotorFeedforward feedforward;
  SparkPIDController climbPID;

  public Climber() {
    feedforward = new SimpleMotorFeedforward(kClimber.kS, kClimber.kV, kClimber.kA);
    climbMotor = new CANSparkMax(kClimber.climberID, MotorType.kBrushless);
    feedforward = new SimpleMotorFeedforward(kClimber.kS, kClimber.kV, kClimber.kA);
    climbPID = climbMotor.getPIDController();
    
    climbPID.setP(kClimber.kP);
    climbPID.setD(kClimber.kD);
    
  }

  public Command driveClimbMotor(DoubleSupplier velocity) {
    return this.runOnce(
        () -> {
          
          // -=-=-=- With PID: -=-=-=-
          climbPID.setReference(0, ControlType.kVoltage, 0, feedforward.calculate(velocity.getAsDouble()));
          
          // -=-=-=- Without PID: -=-=-=-
          //climbMotor.setVoltage(feedforward.calculate(velocity.getAsDouble()));

        });
  }
}

