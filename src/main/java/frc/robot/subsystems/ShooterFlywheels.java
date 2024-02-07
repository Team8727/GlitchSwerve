package frc.robot.subsystems;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kShooter.kFlywheels;

public class ShooterFlywheels extends SubsystemBase {

  private CANSparkMax flywheel1;
  private CANSparkMax flywheel2;

  public ShooterFlywheels() {
    flywheel1 = getSparkMax(kFlywheels.flywheel1ID, CANSparkLowLevel.MotorType.kBrushless);
    flywheel2 = getSparkMax(kFlywheels.flywheel2ID, CANSparkLowLevel.MotorType.kBrushless);
    flywheel2.follow(flywheel1, true);
  }

  public Command runRollers(double volts) {
    return this.run(() -> flywheel1.setVoltage(volts));
  }
}
