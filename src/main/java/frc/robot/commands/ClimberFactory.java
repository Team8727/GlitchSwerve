package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kShooter.kPivot.ShooterPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ShooterPivot;

public class ClimberFactory {

  private final Climber climber;
  private final ShooterPivot shooterPivot;

  public ClimberFactory(Climber climber, ShooterPivot shooterPivot) {
    this.climber = climber;
    this.shooterPivot = shooterPivot;
  }

  public Command goUpFully() {

    return climber.climbUp(11);
  }

  public Command ShooterPivotToHome() {
    return shooterPivot
        .goToPositionCommand(ShooterPosition.HOME)
        .unless(() -> climber.getClimbEncoderRotations() > 3);
  }
}
