package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.utilities.Characterizable;
import java.util.Set;

public class SysIdRoutines {

  @SuppressWarnings("unused")
  private Set<Characterizable> subsystems;

  // Hashmaps for both storing routines and for the selector
  private final SendableChooser<Characterizable> subsystemSelector =
      new SendableChooser<Characterizable>();
  private final SendableChooser<SysIdType> typeSelector = new SendableChooser<SysIdType>();
  private final SendableChooser<String> testSelector = new SendableChooser<String>();

  private SysIdRoutine currentRoutine;

  // Routine Types
  public static enum SysIdType {
    LINEAR,
    ANGULAR,
    LINEARALTERNATE
  }

  public SysIdRoutines(Set<Characterizable> subsystems) {
    this.subsystems = subsystems;

    subsystemSelector.setDefaultOption("No Subsystem", null);
    for (var subsystem : subsystems) {
      subsystemSelector.addOption(subsystem.getName(), subsystem);
    }

    testSelector.addOption("Forward Static", "forwardStatic");
    testSelector.addOption("Reverse Static", "reverseStatic");
    testSelector.addOption("Forward Dynamic", "forwardDynamic");
    testSelector.addOption("Reverse Dynamic", "reverseDynamic");

    for (SysIdType type : SysIdType.values()) {
      typeSelector.addOption(type.name(), type);
    }

    var sysIdTab = Shuffleboard.getTab("SysId");
    sysIdTab.add("Subsystem", subsystemSelector);
    sysIdTab.add("Test Direction", testSelector);
    sysIdTab.add("Test Type", typeSelector);

    subsystemSelector.onChange(
        subsystem -> currentRoutine = subsystem.getRoutine(typeSelector.getSelected()));
    typeSelector.onChange(
        type -> currentRoutine = subsystemSelector.getSelected().getRoutine(type));
  }

  public Command getCommand() {
    if (subsystemSelector.getSelected() == null || currentRoutine == null) return Commands.none();

    switch (testSelector.getSelected()) {
      case "forwardStatic":
        return currentRoutine.quasistatic(Direction.kForward);

      case "reverseStatic":
        return currentRoutine.quasistatic(Direction.kReverse);

      case "forwardDynamic":
        return currentRoutine.dynamic(Direction.kForward);

      case "reverseDynamic":
        return currentRoutine.dynamic(Direction.kReverse);

      default:
        return Commands.none();
    }
  }
}
