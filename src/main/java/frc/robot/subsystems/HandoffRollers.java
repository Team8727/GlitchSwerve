package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kShooter.kHandoffRollers;
import monologue.Annotations.Log;
import monologue.Logged;

public class HandoffRollers extends SubsystemBase implements Logged {
  private final TalonSRX rollerTalonSRX = new TalonSRX(kHandoffRollers.canID);
  private final DigitalInput upperSensor = new DigitalInput(kHandoffRollers.upperSensorPort);
  private final DigitalInput lowerSensor = new DigitalInput(kHandoffRollers.lowerSensorPort);
  private boolean hasPiece = false;

  public HandoffRollers() {
    rollerTalonSRX.setInverted(kHandoffRollers.inverted);
    Shuffleboard.getTab("HandoffRollers").addBoolean("Upper Sensor", this::getUpperSensor);
  }

  public void setVoltage(double voltage) {
    rollerTalonSRX.set(ControlMode.PercentOutput, voltage / 12.0);
  }

  public boolean hasPiece() {
    return hasPiece;
  }

  public boolean getLowerSensor() {
    return !lowerSensor.get();
  }

  public boolean getUpperSensor() {
    return !upperSensor.get();
  }

  public Command feedShooterCommand() {
    return this.startEnd(() -> setVoltage(kHandoffRollers.shooterFeedVoltage), () -> setVoltage(0))
        .withTimeout(kHandoffRollers.shooterFeedTime);
  }

  public Command intakeCommand() {
    return this.startEnd(
            () -> setVoltage(kHandoffRollers.intakeVoltage),
            () -> {
              setVoltage(0);
              if (getUpperSensor()) hasPiece = true;
            })
        .until(this::getUpperSensor);
  }

  @Log.NT
  public double getVoltage() {
    return rollerTalonSRX.getMotorOutputVoltage();
  }
}
