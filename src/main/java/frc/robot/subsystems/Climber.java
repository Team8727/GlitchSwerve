package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimber;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase {

  CANSparkMax climbMotor;

  public Climber() {
    climbMotor = new CANSparkMax(kClimber.climberID, MotorType.kBrushless);
  }

  public Command driveClimbMotor(DoubleSupplier supplyVolts) {
    return this.runOnce(
        () -> {
          climbMotor.setVoltage(supplyVolts.getAsDouble());
        });
  }
}
