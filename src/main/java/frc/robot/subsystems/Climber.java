package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimber;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase {

  CANSparkMax climbMotor;
  SimpleMotorFeedforward feedforward;

  public Climber() {
    feedforward = new SimpleMotorFeedforward(kClimber.kS, kClimber.kV, kClimber.kA);
    climbMotor = new CANSparkMax(kClimber.climberID, MotorType.kBrushless);
  }

  public Command driveClimbMotor(DoubleSupplier velocity) {
    return this.runOnce(
        () -> {
          climbMotor.setVoltage(feedforward.calculate(velocity.getAsDouble()));
        });
  }
}
