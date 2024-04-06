package frc.robot.subsystems;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
  private RelativeEncoder insideEncoder;

  private boolean hasPiece;
  private boolean indexing;

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
    intakeMotor.setOpenLoopRampRate(kRollers.rampRate);
    intakeMotor.burnFlash();

    pieceCheck = new DigitalInput(kRollers.sensorChannel);
    insideEncoder = intakeMotor.getEncoder();
    hasPiece = false;
    indexing = false;
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
  public boolean isIndexing() {
    return indexing;
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
    return this.runOnce(() -> runRollers(kRollers.intakeVoltage))
        .andThen(Commands.waitUntil(this::getPieceCheck))
        .finallyDo(
            (interrupted) -> {
              runRollers(0);
              if (!interrupted) indexing = true;
            });
  }

  public Command index() {
    return this.run(() -> runRollers(kRollers.intakeVoltage))
        .withTimeout(kRollers.intakeTime)
        .finallyDo(
            () -> {
              runRollers(0);
              indexing = false;
              hasPiece = true;
            })
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command softIntakeFromAmp() {
    return this.startEnd(() -> runRollers(kRollers.softIntakeFromAmpVoltage), () -> runRollers(0));
  }

  public Command outtakeCommand() {
    return this.startEnd(
        () -> runRollers(kRollers.outtakeVoltage),
        () -> {
          runRollers(0);
          hasPiece = false;
        });
  }

  public Command unjamIntake() {
    return this.startEnd(() -> runRollers(kRollers.intakeVoltage), () -> runRollers(0))
        .withTimeout(kRollers.ejectIntakeTime);
  }

  public Command eject() {
    return this.startEnd(() -> runRollers(-kRollers.intakeVoltage), () -> runRollers(0))
        .withTimeout(kRollers.ejectIntakeTime);
  }
}
