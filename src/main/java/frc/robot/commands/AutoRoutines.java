package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import java.util.LinkedHashMap;
import java.util.Map;

public class AutoRoutines {
  @SuppressWarnings("unused")
  private final Swerve swerve;

  @SuppressWarnings("unused")
  private final LinkedHashMap<String, PathPlannerPath> paths =
      new LinkedHashMap<String, PathPlannerPath>();

  private final LinkedHashMap<String, Command> routines = new LinkedHashMap<String, Command>();
  private final SendableChooser<Command> selector = new SendableChooser<Command>();

  public AutoRoutines(Swerve swerve) {
    this.swerve = swerve;

    loadPaths();
    loadCommands();
    loadRoutines();
    populateSendable();
  }

  /* Add paths to the hashmap using this format:
     paths.put("<Name>", PathPlannerPath.fromPathFile("<path file name>"));
     ex:
     paths.put("Crazy auto", swerve.followPathWithEventsCommand(paths.get("crazy_auto")));
  */
  private void loadPaths() {}

  // Add commands to PathPlanner in this form:
  // NamedCommands.registerCommand("<Name>", <command>);
  // Must match the naming in the PathPlanner app
  private void loadCommands() {}

  /* Add routines to the hashmap using this format:
     routines.put("<Name>", <Command to run>);
     ex:
     routines.put("Crazy auto", swerve.followPathWithEventsCommand(paths.get("Crazy auto")));
  */
  private void loadRoutines() {
    routines.put("No Auto", Commands.waitSeconds(0));
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
