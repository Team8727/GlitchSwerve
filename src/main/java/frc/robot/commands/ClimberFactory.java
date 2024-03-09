package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kShooter.kPivot.ShooterPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ShooterPivot;

public class ClimberFactory {

  private final Climber climber;
  private final ShooterPivot shooterPivot;
  private ShuffleboardTab tab = Shuffleboard.getTab("Active Configs");

  public ClimberFactory(Climber climber, ShooterPivot shooterPivot) {
    this.climber = climber;
    this.shooterPivot = shooterPivot;
    tab.add("EndGame", Commands.runOnce(() -> climber.setEndGame(true)));
  }

  public Command setEndGame() {
    return Commands.runOnce(() -> climber.setEndGame(true));
  }

  public Command goUpFully() {

    return Commands.runOnce(
            () -> {
              shooterPivot.goToPositionCommand(ShooterPosition.CLIMB);
              climber.climbUp(3);
            })
        .unless(() -> !climber.getEndGame());
  }
}
