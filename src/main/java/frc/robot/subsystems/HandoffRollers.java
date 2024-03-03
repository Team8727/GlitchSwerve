package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kShooter.kHandoffRollers;
import monologue.Annotations.Log;
import monologue.Logged;

public class HandoffRollers extends SubsystemBase implements Logged {
  private TalonSRX rollerTalonSRX = new TalonSRX(kHandoffRollers.canID);

  public HandoffRollers() {
    rollerTalonSRX.setInverted(kHandoffRollers.inverted);
  }

  public void setVoltage(double voltage) {
    rollerTalonSRX.set(ControlMode.PercentOutput, voltage / 12.0);
  }

  public Command feedShooterCommand() {
    return this.startEnd(() -> setVoltage(kHandoffRollers.shooterFeedVoltage), () -> setVoltage(0))
        .withTimeout(kHandoffRollers.shooterFeedTime);
  }

  @Log.NT
  public double getVoltage() {
    return rollerTalonSRX.getMotorOutputVoltage();
  }
}
