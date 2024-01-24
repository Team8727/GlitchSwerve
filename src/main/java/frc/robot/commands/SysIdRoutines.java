package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;
import java.util.LinkedHashMap;
import java.util.Map;

public class SysIdRoutines {

  @SuppressWarnings("unused")
  private final Swerve swerve;

  // Hashmaps for both storing routines and for the selector
  private final LinkedHashMap<String, Command> routines = new LinkedHashMap<String, Command>();
  private final SendableChooser<Command> selector = new SendableChooser<Command>();

  public SysIdRoutines(Swerve swerve) {
    this.swerve = swerve;

    loadRoutines();
    populateSendable();
  }

  // Command Methods that return either a quasistatic or dynamic routine for the SysIdProcess
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction, kSwerve.sysIdType type) {
    if (type == kSwerve.sysIdType.ANGULAR) {
      return swerve.getAngularRoutine().quasistatic(direction);
    } else {
      return swerve.getLinearRoutine().quasistatic(direction);
    }
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction, kSwerve.sysIdType type) {
    if (type == kSwerve.sysIdType.ANGULAR) {
      return swerve.getAngularRoutine().dynamic(direction);
    } else {
      return swerve.getLinearRoutine().dynamic(direction);
    }
  }

  // Loads all necessary routine permutations into a Hashmap
  private void loadRoutines() {
    routines.put("No Routine", Commands.waitSeconds(100));

    // Linear Routines
    routines.put(
        "linearForwardQuasistatic", sysIdQuasistatic(Direction.kForward, kSwerve.sysIdType.LINEAR));
    routines.put(
        "linearReverseQuasistatic", sysIdQuasistatic(Direction.kReverse, kSwerve.sysIdType.LINEAR));
    routines.put(
        "linearForwardDynamic", sysIdDynamic(Direction.kForward, kSwerve.sysIdType.LINEAR));
    routines.put(
        "linearReverseDynamic", sysIdDynamic(Direction.kReverse, kSwerve.sysIdType.LINEAR));
    // Angular Routines
    routines.put(
        "angularForwardQuasistatic",
        sysIdQuasistatic(Direction.kForward, kSwerve.sysIdType.ANGULAR));
    routines.put(
        "angularReverseQuasistatic",
        sysIdQuasistatic(Direction.kReverse, kSwerve.sysIdType.ANGULAR));
    routines.put(
        "angularForwardDynamic", sysIdDynamic(Direction.kForward, kSwerve.sysIdType.ANGULAR));
    routines.put(
        "angularReverseDynamic", sysIdDynamic(Direction.kReverse, kSwerve.sysIdType.ANGULAR));
  }

  // Adds all the Commands to the sendable chooser
  private void populateSendable() {
    selector.setDefaultOption("No Routine", routines.get("No Routine"));
    for (Map.Entry<String, Command> entry : routines.entrySet()) {
      selector.addOption(entry.getKey(), entry.getValue());
    }
  }

  // Retuns the SendableChooser of commands
  public SendableChooser<Command> getSelector() {
    return selector;
  }
}
