package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kIntakeShooter.kHandOff;
import frc.robot.Constants.kIntakeShooter.kShootAmp;
import frc.robot.Constants.kIntakeShooter.kShootSpeaker;
import frc.robot.Constants.kShooter.kPivot.Position;
import frc.robot.subsystems.HandoffRollers;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.ShooterFlywheels;
import frc.robot.subsystems.ShooterPivot;

public class IntakeShooter {

  private final HandoffRollers handoffRollers;
  private final IntakePivot intakePivot;
  private final IntakeRollers intakeRollers;
  private final ShooterFlywheels shooterFlywheels;
  private final ShooterPivot shooterPivot;

  public IntakeShooter(
      HandoffRollers handoffRollers,
      IntakePivot intakePivot,
      IntakeRollers intakeRollers,
      ShooterFlywheels shooterFlywheels,
      ShooterPivot shooterPivot) {
    this.handoffRollers = handoffRollers;
    this.intakePivot = intakePivot;
    this.intakeRollers = intakeRollers;
    this.shooterFlywheels = shooterFlywheels;
    this.shooterPivot = shooterPivot;
  }

  public Command intakeProcess() {
    return intakeRollers
        .intake()
        .deadlineWith(intakePivot.setIntakeDown(true))
        .andThen(intakeRollers.index());
  }

  public Command autoIntake() {
    return intakeRollers
        .intake()
        .deadlineWith(intakePivot.setIntakeDown(true))
        .andThen(
            intakeRollers
                .index()
                .alongWith(intakePivot.setIntakeDown(false).until(intakePivot::isHome)))
        .andThen(handOff());
  }

  public Command shootSpeaker() {
    return shooterFlywheels
        .shootTest(kShootSpeaker.shootVoltage)
        .raceWith(
            Commands.waitSeconds(kShootSpeaker.delay)
                .andThen(
                    handoffRollers
                        .feedShooterCommand()
                        .deadlineWith(intakeRollers.outtakeCommand())));
  }

  public Command handOff() {
    return intakeRollers
        .outtakeCommand()
        .raceWith(handoffRollers.intakeCommand())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withTimeout(kHandOff.timeout);
  }

  public Command shootAmp() {
    return shooterPivot
        .goToPositionCommand(Position.AMP)
        .andThen(shooterFlywheels.shootTest(kShootAmp.shootVoltage));
  }
}
