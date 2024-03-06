package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Swerve;
import java.util.LinkedHashMap;
import java.util.Map;

public class AutoRoutines {
  private final Swerve swerve;
  private final ShooterPivot shooterPivot;
  private final IntakeShooter intakeShooterCommands;

  private final LinkedHashMap<String, PathPlannerPath> paths =
      new LinkedHashMap<String, PathPlannerPath>();

  private final LinkedHashMap<String, Command> routines = new LinkedHashMap<String, Command>();
  private final SendableChooser<Command> selector = new SendableChooser<Command>();

  public AutoRoutines(
      Swerve swerve, ShooterPivot shooterPivot, IntakeShooter intakeShooterCommands) {
    this.swerve = swerve;
    this.shooterPivot = shooterPivot;
    this.intakeShooterCommands = intakeShooterCommands;

    loadCommands();
    loadPaths();
    loadRoutines();
    populateSendable();
  }

  /* Add paths to the hashmap using this format:
     paths.put("<Name>", PathPlannerPath.fromPathFile("<path file name>"));
     ex:
     paths.put("Crazy auto", swerve.followPathWithEventsCommand(paths.get("crazy_auto")));
  */
  private void loadPaths() {
    paths.put("fourNote1", PathPlannerPath.fromChoreoTrajectory("four note.1"));
    paths.put("fourNote2", PathPlannerPath.fromChoreoTrajectory("four note.2"));
    paths.put("fourNote3", PathPlannerPath.fromChoreoTrajectory("four note.3"));
    paths.put("testIntake", PathPlannerPath.fromChoreoTrajectory("testIntake"));
  }

  // Add commands to PathPlanner in this form:
  // NamedCommands.registerCommand("<Name>", <command>);
  // Must match the naming in the PathPlanner app
  private void loadCommands() {
    NamedCommands.registerCommand("intake", intakeShooterCommands.autoIntake());
    NamedCommands.registerCommand("shootSpeaker", intakeShooterCommands.shootSpeaker());
  }

  /* Add routines to the hashmap using this format:
     routines.put("<Name>", <Command to run>);
     ex:
     routines.put("Crazy auto", swerve.followPathWithEventsCommand(paths.get("Crazy auto")));
  */
  private void loadRoutines() {
    routines.put("No Auto", Commands.waitSeconds(0));
    routines.put(
        "fourNote",
        intakeShooterCommands
            .shootSpeaker()
            .andThen(
                swerve
                    .followPathCommand(paths.get("fourNote1"), true)
                    .alongWith(intakeShooterCommands.autoIntake()))
            .andThen(intakeShooterCommands.shootSpeaker())
            .andThen(
                swerve
                    .followPathCommand(paths.get("fourNote2"), true)
                    .alongWith(intakeShooterCommands.autoIntake()))
            .andThen(intakeShooterCommands.shootSpeaker())
            .andThen(
                swerve
                    .followPathCommand(paths.get("fourNote3"), true)
                    .alongWith(intakeShooterCommands.autoIntake()))
            .andThen(intakeShooterCommands.shootSpeaker()));
    routines.put(
        "test",
        swerve
            .followPathCommand(paths.get("testIntake"), true)
            .andThen(Commands.print("Finished swerve"))
            .andThen(Commands.waitSeconds(1))
            .andThen(intakeShooterCommands.shootSpeaker())
            .andThen(Commands.print("Made shot")));
  }

  // Adds all the Commands to the sendable chooser
  private void populateSendable() {
    selector.setDefaultOption("No Auto", routines.get("No Auto"));
    for (Map.Entry<String, Command> entry : routines.entrySet()) {
      selector.addOption(entry.getKey(), entry.getValue());
    }
  }

  // Retuns the SendableChooser of commands
  public SendableChooser<Command> getSelector() {
    return selector;
  }
}
