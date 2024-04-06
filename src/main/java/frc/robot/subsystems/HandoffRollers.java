package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    rollerTalonSRX.setNeutralMode(NeutralMode.Brake);
    Shuffleboard.getTab("Driver Info").addBoolean("Upper Sensor", this::getUpperSensor);
    Shuffleboard.getTab("Driver Info").addBoolean("Has Piece", this::hasPiece);
  }

  public void setVoltage(double voltage) {
    rollerTalonSRX.set(ControlMode.PercentOutput, voltage / 12.0);
  }

  @Log.NT
  public boolean hasPiece() {
    return hasPiece;
  }

  public boolean getLowerSensor() {
    return lowerSensor.get();
  }

  @Log.NT
  public boolean getUpperSensor() {
    return !upperSensor.get();
  }

  public Command feedShooterCommand() {
    return this.startEnd(
            () -> setVoltage(kHandoffRollers.shooterFeedVoltage),
            () -> {
              setVoltage(0);
              hasPiece = false;
            })
        .withTimeout(kHandoffRollers.shooterFeedTime);
  }

  public Command intakeSource() {
    return this.startEnd(
            () -> setVoltage(kHandoffRollers.sourceIntakeVoltage),
            () -> {
              setVoltage(0);
              hasPiece = true;
            })
        .withTimeout(kHandoffRollers.sourceIntakeFeedTime);
  }

  public Command intakeCommand() {
    return this.runOnce(() -> setVoltage(kHandoffRollers.intakeVoltage))
        .andThen(Commands.waitUntil(this::getUpperSensor))
        .finallyDo(
            (interrupted) -> {
              setVoltage(0);
              if (getUpperSensor()) hasPiece = true;
            });
  }

  public Command outtakeCommand() {
    return this.startEnd(
        () -> setVoltage(-kHandoffRollers.intakeVoltage),
        () -> {
          setVoltage(0);
          hasPiece = false;
        });
  }

  @Log.NT
  public double getVoltage() {
    return rollerTalonSRX.getMotorOutputVoltage();
  }
}
