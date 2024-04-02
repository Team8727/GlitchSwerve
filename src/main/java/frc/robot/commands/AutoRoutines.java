package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.ShooterFlywheels;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Swerve;
import java.util.LinkedHashMap;
import java.util.Map;

public class AutoRoutines {
  private final Swerve swerve;

  @SuppressWarnings("unused")
  private final ShooterPivot shooterPivot;

  private final ShooterFlywheels flywheels;
  private final IntakeShooter intakeShooterCommands;
  private final IntakePivot intakePivot;

  private final LinkedHashMap<String, PathPlannerPath> paths =
      new LinkedHashMap<String, PathPlannerPath>();

  private final LinkedHashMap<String, Command> routines = new LinkedHashMap<String, Command>();
  private final SendableChooser<Command> selector = new SendableChooser<Command>();

  public AutoRoutines(
      Swerve swerve,
      ShooterFlywheels flywheels,
      ShooterPivot shooterPivot,
      IntakeShooter intakeShooterCommands,
      IntakePivot intakePivot) {
    this.swerve = swerve;
    this.flywheels = flywheels;
    this.shooterPivot = shooterPivot;
    this.intakeShooterCommands = intakeShooterCommands;
    this.intakePivot = intakePivot;

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
    paths.put("fiveNote1", PathPlannerPath.fromChoreoTrajectory("five note.1"));
    paths.put("fiveNote2", PathPlannerPath.fromChoreoTrajectory("five note.2"));
    paths.put("fiveNote3", PathPlannerPath.fromChoreoTrajectory("five note.3"));
    paths.put("fiveNote4", PathPlannerPath.fromChoreoTrajectory("five note.4"));
    paths.put("shootTaxiLeft", PathPlannerPath.fromChoreoTrajectory("shootTaxiLeft"));
    paths.put("shootTaxiRight", PathPlannerPath.fromChoreoTrajectory("shootTaxiRight"));
    paths.put("testIntake", PathPlannerPath.fromChoreoTrajectory("testIntake"));
    paths.put("far1", PathPlannerPath.fromChoreoTrajectory("far.1"));
    paths.put("far2", PathPlannerPath.fromChoreoTrajectory("far.2"));
  }

  // Add commands to PathPlanner in this form:
  // NamedCommands.registerCommand("<Name>", <command>);
  // Must match the naming in the PathPlanner app
  private void loadCommands() {
    NamedCommands.registerCommand("intake", intakeShooterCommands.autoIntake());
    NamedCommands.registerCommand("shootSpeaker", intakeShooterCommands.autoShoot());
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
        flywheels
            .shootVoltage(10)
            .raceWith(
                Commands.waitSeconds(0.5)
                    .andThen(
                        intakeShooterCommands
                            .autoShoot()
                            .andThen(
                                swerve
                                    .followPathCommand(paths.get("fourNote1"), true, true)
                                    .alongWith(
                                        Commands.waitSeconds(0.1)
                                            .andThen(
                                                intakeShooterCommands.autoIntake().withTimeout(2))))
                            .andThen(intakeShooterCommands.autoShoot())
                            .andThen(
                                swerve
                                    .followPathCommand(paths.get("fourNote2"), true)
                                    .alongWith(intakeShooterCommands.autoIntake().withTimeout(2.4)))
                            .andThen(intakeShooterCommands.autoShoot())
                            .andThen(
                                swerve
                                    .followPathCommand(paths.get("fourNote3"), true)
                                    .alongWith(intakeShooterCommands.autoIntake()))
                            .andThen(intakeShooterCommands.autoShoot()))));
    routines.put(
        "fiveNote",
        flywheels
            .shootVoltage(10)
            .raceWith(
                Commands.waitSeconds(0.5)
                    .andThen(
                        intakeShooterCommands
                            .autoShoot()
                            .andThen(
                                swerve
                                    .followPathCommand(paths.get("fiveNote1"), true, true)
                                    .deadlineWith(
                                        Commands.waitSeconds(0.1)
                                            .andThen(intakeShooterCommands.autoIntake())))
                            .andThen(intakeShooterCommands.autoShoot())
                            .andThen(
                                swerve
                                    .followPathCommand(paths.get("fiveNote2"), true, true)
                                    .deadlineWith(intakeShooterCommands.autoIntake()))
                            .andThen(intakeShooterCommands.autoShoot())
                            .andThen(
                                swerve
                                    .followPathCommand(paths.get("fiveNote3"), true, true)
                                    .deadlineWith(intakeShooterCommands.autoIntake()))
                            .andThen(intakeShooterCommands.autoShoot())
                            .andThen(
                                swerve
                                    .followPathCommand(paths.get("fiveNote4"), true, true)
                                    .deadlineWith(intakeShooterCommands.autoIntake()))
                            .andThen(intakeShooterCommands.autoShoot()))));
    routines.put(
        "far",
        flywheels
            .shootVoltage(10)
            .raceWith(
                Commands.waitSeconds(0.5)
                    .andThen(
                        intakeShooterCommands
                            .autoShoot()
                            .andThen(
                                swerve
                                    .followPathCommand(paths.get("far1"), true, true)
                                    .deadlineWith(
                                        Commands.waitSeconds(0.1)
                                            .andThen(intakeShooterCommands.autoIntake())))
                            .andThen(intakeShooterCommands.autoShoot())
                            .andThen(
                                swerve
                                    .followPathCommand(paths.get("far2"), true)
                                    .deadlineWith(intakeShooterCommands.autoIntake()))
                            .andThen(intakeShooterCommands.autoShoot()))));
    routines.put(
        "test",
        swerve
            .followPathCommand(paths.get("testIntake"), true)
            .andThen(Commands.print("Finished swerve"))
            .andThen(Commands.waitSeconds(1))
            .andThen(intakeShooterCommands.shootSpeaker())
            .andThen(Commands.print("Made shot")));
    routines.put(
        "shootTaxiLeft",
        flywheels
            .shootVoltage(10)
            .raceWith(
                Commands.waitSeconds(0.5)
                    .andThen(swerve.followPathCommand(paths.get("shootTaxiLeft"), true))));
    routines.put(
        "shootTaxiRight",
        flywheels
            .shootVoltage(10)
            .raceWith(
                Commands.waitSeconds(0.5)
                    .andThen(swerve.followPathCommand(paths.get("shootTaxiRight"), true))));
    routines.put("shootOnly", flywheels.shootVoltage(10));
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
