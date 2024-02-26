package frc.robot.subsystems;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIntake.kRollers;
import frc.robot.utilities.SparkConfigurator.LogData;
import java.util.Set;

public class IntakeRollers extends SubsystemBase {

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

  public Command runRollers(double volts) {
    return this.run(() -> intakeMotor.setVoltage(volts));
  }

  public boolean getPieceCheck() {
    return !pieceCheck.get(); // Invert because of sensor
  }

  private void setHasPiece(boolean piece) {
    hasPiece = piece;
  }

  public boolean pieceState() {
    return hasPiece;
  }

  public Command intakeCommand() {
    return runRollers(kRollers.intakeVoltage)
        .until(() -> getPieceCheck())
        .finallyDo(() -> setHasPiece(true));
  }

  public Command outtakeCommand() {
    return runRollers(kRollers.outtakeVoltage)
        .until(() -> getPieceCheck())
        .andThen(runRollers(kRollers.outtakeVoltage).withTimeout(kRollers.delayForOuttake));
  }
}
