package frc.robot.subsystems;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIntake.kRollers;
import frc.robot.utilities.SparkConfigurator.LogData;
import java.util.Set;
import monologue.Annotations.Log;
import monologue.Logged;

public class IntakeRollers extends SubsystemBase implements Logged {

  private CANSparkMax intakeMotor;
  private DigitalInput pieceCheck;
  private RelativeEncoder insideEncoder;

  private boolean hasPiece;

  public IntakeRollers() {
    intakeMotor =
        getSparkMax(
            kRollers.rollerMotorID,
            CANSparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(LogData.CURRENT, LogData.VOLTAGE, LogData.POSITION));
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setInverted(kRollers.invert);
    intakeMotor.setSmartCurrentLimit(kRollers.currentLimit);
    intakeMotor.burnFlash();

    pieceCheck = new DigitalInput(kRollers.sensorChannel);
    insideEncoder = intakeMotor.getEncoder();
    hasPiece = false;
  }

  public void runRollers(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Log.NT
  public boolean getPieceCheck() {
    return !pieceCheck.get();
  }

  @Log.NT
  public boolean hasPiece() {
    return hasPiece;
  }

  @Log.NT
  public double getCurrent() {
    return intakeMotor.getOutputCurrent();
  }

  @Log.NT
  public double getInsideEncoder() {
    return insideEncoder.getPosition();
  }

  @Log.NT
  public double getAppliedVoltage() {
    return intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
  }

  public Command intake() {
    return this.startEnd(() -> runRollers(kRollers.intakeVoltage1), () -> runRollers(0))
        .until(this::getPieceCheck);
  }

  public Command index() {
    return this.run(() -> runRollers(kRollers.intakeVoltage2))
        .withTimeout(kRollers.intakeDelay)
        .andThen(
            this.startEnd(() -> runRollers(kRollers.intakeVoltage3), () -> hasPiece = true)
                .until(() -> getCurrent() > kRollers.currentThreshold)
                .withTimeout(kRollers.stage3Timeout))
        .finallyDo(() -> runRollers(0));
  }

  public Command intakeThreeStageCommand() {
    return this.run(() -> runRollers(kRollers.intakeVoltage1))
        .until(this::getPieceCheck)
        .andThen(
            this.run(() -> runRollers(kRollers.intakeVoltage2)).withTimeout(kRollers.intakeDelay))
        .andThen(
            this.startEnd(() -> runRollers(kRollers.intakeVoltage3), () -> hasPiece = true)
                .until(() -> getCurrent() > kRollers.currentLimit)
                .withTimeout(2))
        .finallyDo(() -> runRollers(0));
  }

  public Command outtakeCommand() {
    return this.startEnd(
        () -> runRollers(kRollers.outtakeVoltage),
        () -> {
          runRollers(0);
          hasPiece = false;
        });
  }
}
