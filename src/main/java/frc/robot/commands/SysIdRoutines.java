package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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

  // Loads all necessary routine permutations into a Hashmap
  private void loadRoutines() {
    routines.put("No Routine", Commands.waitSeconds(100));

    // Linear Swerve Routines
    routines.put(
        "swerveLinearForwardQuasistatic",
        swerve.getLinearRoutine().quasistatic(Direction.kForward));
    routines.put(
        "swerveLinearReverseQuasistatic",
        swerve.getLinearRoutine().quasistatic(Direction.kReverse));
    routines.put("linearForwardDynamic", swerve.getLinearRoutine().dynamic(Direction.kForward));
    routines.put("linearReverseDynamic", swerve.getLinearRoutine().dynamic(Direction.kReverse));

    // Angular Swerve Routines
    routines.put(
        "swerveAngularForwardQuasistatic",
        swerve.getAngularRoutine().quasistatic(Direction.kForward));
    routines.put(
        "swerveAngularReverseQuasistatic",
        swerve.getAngularRoutine().quasistatic(Direction.kForward));
    routines.put("angularForwardDynamic", swerve.getAngularRoutine().dynamic(Direction.kForward));
    routines.put("angularReverseDynamic", swerve.getAngularRoutine().dynamic(Direction.kReverse));

    // IntakePivtot
    routines.put(
        "intakePivotForwardQuasistatic",
        intakePivot.getAngularRoutine().quasistatic(Direction.kForward));
    routines.put(
        "intakePivotReverseQuasistatic",
        intakePivot.getAngularRoutine().quasistatic(Direction.kReverse));
    routines.put(
        "intakePivotForwardDynamic", intakePivot.getAngularRoutine().dynamic(Direction.kForward));
    routines.put(
        "intakePivotReverseDynamic", intakePivot.getAngularRoutine().dynamic(Direction.kReverse));

    // ShooterPivot
    routines.put(
        "shooterPivotForwardQuasistatic",
        shooterPivot.getAngularRoutine().quasistatic(Direction.kForward));
    routines.put(
        "shooterPivotReverseQuasistatic",
        shooterPivot.getAngularRoutine().quasistatic(Direction.kReverse));
    routines.put(
        "shooterPivotForwardDynamic", shooterPivot.getAngularRoutine().dynamic(Direction.kForward));
    routines.put(
        "shooterPivotReverseDynamic", shooterPivot.getAngularRoutine().dynamic(Direction.kReverse));

    // ShooterFlywheels
    routines.put(
        "shooterFlywheelsForwardQuasistatic",
        shooterFlywheels.getAngularRoutine().quasistatic(Direction.kForward));
    routines.put(
        "shooterFlywheelsReverseQuasistatic",
        shooterFlywheels.getAngularRoutine().quasistatic(Direction.kReverse));
    routines.put(
        "shooterFlywheelsForwardDynamic",
        shooterFlywheels.getAngularRoutine().dynamic(Direction.kForward));
    routines.put(
        "shooterFlywheelsReverseDynamic",
        shooterFlywheels.getAngularRoutine().dynamic(Direction.kReverse));
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
