package frc.robot.subsystems;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIntake.kRollers;
import frc.robot.utilities.SparkConfigurator.LogData;
import java.util.Set;
import monologue.Annotations.Log;
import monologue.Logged;

public class IntakeRollers extends SubsystemBase implements Logged {

  private CANSparkMax intakeMotor;
  private DigitalInput pieceCheck;

  private boolean hasPiece;

  public IntakeRollers() {
    intakeMotor =
        getSparkMax(
            kRollers.rollerMotorID,
            CANSparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(LogData.CURRENT, LogData.VOLTAGE));
    intakeMotor.setIdleMode(IdleMode.kBrake);
    pieceCheck = new DigitalInput(kRollers.sensorChannel);
    hasPiece = false;
  }

  public void runRollers(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Log.NT
  public boolean getPieceCheck() {
    return !pieceCheck.get();
  }

  private void setHasPiece(boolean piece) {
    hasPiece = piece;
  }

  public boolean pieceState() {
    return hasPiece;
  }

  public Command intakeCommand() {
    return this.run(() -> runRollers(kRollers.intakeVoltage))
        .until(this::getPieceCheck)
        .andThen(Commands.waitSeconds(kRollers.intakeDelay))
        .finallyDo(() -> runRollers(0));
  }
}
