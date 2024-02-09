package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.ShooterFlywheels;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Swerve;
import java.util.LinkedHashMap;
import java.util.Map;

public class SysIdRoutines {

  private Swerve swerve;
  private IntakePivot intakePivot;
  private ShooterFlywheels shooterFlywheels;
  private ShooterPivot shooterPivot;

  // Hashmaps for both storing routines and for the selector
  private final LinkedHashMap<String, Command> routines = new LinkedHashMap<String, Command>();
  private final SendableChooser<Command> selector = new SendableChooser<Command>();

  // When only swerve exists
  public SysIdRoutines(Swerve swerve) {
    this.swerve = swerve;

    loadRoutines();
    populateSendable();
  }

  // When both the swerve and intake exist
  public SysIdRoutines(Swerve swerve, IntakePivot intakePivot) {
    this.swerve = swerve;
    this.intakePivot = intakePivot;
    loadRoutines();
    populateSendable();
  }

  // When Swerve Intake and Shooter All exist
  public SysIdRoutines(
      Swerve swerve,
      IntakePivot intakePivot,
      ShooterFlywheels shooterFlywheels,
      ShooterPivot shooterPivot) {
    this.swerve = swerve;
    this.intakePivot = intakePivot;
    this.shooterFlywheels = shooterFlywheels;
    this.shooterPivot = shooterPivot;
    loadRoutines();
    populateSendable();
  }

  // Command Methods that return either a quasistatic or dynamic routine for the SysIdProcess
  private Command sysIdQuasistatic(
      SysIdRoutine.Direction direction, Constants.sysIdType type, Constants.subsystems subsystem) {
    if (subsystem == Constants.subsystems.SWERVE) {
      if (type == Constants.sysIdType.ANGULAR) {
        return swerve.getAngularRoutine().quasistatic(direction);
      } else {
        return swerve.getLinearRoutine().quasistatic(direction);
      }
    } else if (subsystem == Constants.subsystems.INTAKEPIVOT) {
      return intakePivot.getAngularRoutine().quasistatic(direction);
    } else if (subsystem == Constants.subsystems.SHOOTERPIVOT) {
      return shooterPivot.getAngularRoutine().quasistatic(direction);
    } else if (subsystem == Constants.subsystems.SHOOTERFLYWHEELS) {
      return shooterFlywheels.getAngularRoutine().quasistatic(direction);
    } else {
      return new WaitCommand(100);
    }
  }

  private Command sysIdDynamic(
      SysIdRoutine.Direction direction, Constants.sysIdType type, Constants.subsystems subsystem) {
    if (subsystem == Constants.subsystems.SWERVE) {
      if (type == Constants.sysIdType.ANGULAR) {
        return swerve.getAngularRoutine().dynamic(direction);
      } else {
        return swerve.getLinearRoutine().dynamic(direction);
      }
    } else if (subsystem == Constants.subsystems.INTAKEPIVOT) {
      return intakePivot.getAngularRoutine().dynamic(direction);
    } else if (subsystem == Constants.subsystems.SHOOTERPIVOT) {
      return shooterPivot.getAngularRoutine().dynamic(direction);
    } else if (subsystem == Constants.subsystems.SHOOTERFLYWHEELS) {
      return shooterFlywheels.getAngularRoutine().dynamic(direction);
    } else {
      return new WaitCommand(100);
    }
  }

  // Loads all necessary routine permutations into a Hashmap
  private void loadRoutines() {
    routines.put("No Routine", Commands.waitSeconds(100));

    // Linear Swerve Routines
    routines.put(
        "swerveLinearForwardQuasistatic",
        sysIdQuasistatic(
            Direction.kForward, Constants.sysIdType.LINEAR, Constants.subsystems.SWERVE));
    routines.put(
        "swerveLinearReverseQuasistatic",
        sysIdQuasistatic(
            Direction.kReverse, Constants.sysIdType.LINEAR, Constants.subsystems.SWERVE));
    routines.put(
        "linearForwardDynamic",
        sysIdDynamic(Direction.kForward, Constants.sysIdType.LINEAR, Constants.subsystems.SWERVE));
    routines.put(
        "linearReverseDynamic",
        sysIdDynamic(Direction.kReverse, Constants.sysIdType.LINEAR, Constants.subsystems.SWERVE));
    // Angular Swerve Routines
    routines.put(
        "swerveAngularForwardQuasistatic",
        sysIdQuasistatic(
            Direction.kForward, Constants.sysIdType.ANGULAR, Constants.subsystems.SWERVE));
    routines.put(
        "swerveAngularReverseQuasistatic",
        sysIdQuasistatic(
            Direction.kReverse, Constants.sysIdType.ANGULAR, Constants.subsystems.SWERVE));
    routines.put(
        "angularForwardDynamic",
        sysIdDynamic(Direction.kForward, Constants.sysIdType.ANGULAR, Constants.subsystems.SWERVE));
    routines.put(
        "angularReverseDynamic",
        sysIdDynamic(Direction.kReverse, Constants.sysIdType.ANGULAR, Constants.subsystems.SWERVE));

    // IntakePivtot
    routines.put(
        "intakePivotForwardQuasistatic",
        sysIdQuasistatic(
            Direction.kForward, Constants.sysIdType.ANGULAR, Constants.subsystems.INTAKEPIVOT));
    routines.put(
        "intakePivotReverseQuasistatic",
        sysIdQuasistatic(
            Direction.kReverse, Constants.sysIdType.ANGULAR, Constants.subsystems.INTAKEPIVOT));
    routines.put(
        "intakePivotForwardDynamic",
        sysIdDynamic(
            Direction.kForward, Constants.sysIdType.ANGULAR, Constants.subsystems.INTAKEPIVOT));
    routines.put(
        "intakePivotReverseDynamic",
        sysIdDynamic(
            Direction.kReverse, Constants.sysIdType.ANGULAR, Constants.subsystems.INTAKEPIVOT));

    // ShooterPivot
    routines.put(
        "shooterPivotForwardQuasistatic",
        sysIdQuasistatic(
            Direction.kForward, Constants.sysIdType.ANGULAR, Constants.subsystems.SHOOTERPIVOT));
    routines.put(
        "shooterPivotReverseQuasistatic",
        sysIdQuasistatic(
            Direction.kReverse, Constants.sysIdType.ANGULAR, Constants.subsystems.SHOOTERPIVOT));
    routines.put(
        "shooterPivotForwardDynamic",
        sysIdDynamic(
            Direction.kForward, Constants.sysIdType.ANGULAR, Constants.subsystems.SHOOTERPIVOT));
    routines.put(
        "shooterPivotReverseDynamic",
        sysIdDynamic(
            Direction.kReverse, Constants.sysIdType.ANGULAR, Constants.subsystems.SHOOTERPIVOT));

    // ShooterFlywheels
    routines.put(
        "shooterFlywheelsForwardQuasistatic",
        sysIdQuasistatic(
            Direction.kForward,
            Constants.sysIdType.ANGULAR,
            Constants.subsystems.SHOOTERFLYWHEELS));
    routines.put(
        "shooterFlywheelsReverseQuasistatic",
        sysIdQuasistatic(
            Direction.kReverse,
            Constants.sysIdType.ANGULAR,
            Constants.subsystems.SHOOTERFLYWHEELS));
    routines.put(
        "shooterFlywheelsForwardDynamic",
        sysIdDynamic(
            Direction.kForward,
            Constants.sysIdType.ANGULAR,
            Constants.subsystems.SHOOTERFLYWHEELS));
    routines.put(
        "shooterFlywheelsReverseDynamic",
        sysIdDynamic(
            Direction.kReverse,
            Constants.sysIdType.ANGULAR,
            Constants.subsystems.SHOOTERFLYWHEELS));
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
