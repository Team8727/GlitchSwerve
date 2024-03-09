// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SimMode;
import frc.robot.Constants.kIntake.kPivot.IntakePosition;
import frc.robot.Constants.kShooter.kPivot.ShooterPosition;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ClimberFactory;
import frc.robot.commands.IntakeShooter;
import frc.robot.commands.SysIdRoutines;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.HandoffRollers;
import frc.robot.subsystems.Indications;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.ShooterFlywheels;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Swerve;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.Set;
import monologue.Logged;
import monologue.Monologue;

public class Robot extends TimedRobot implements Logged {
  private CommandXboxController driverController = new CommandXboxController(0);

  // Subsystems
  private Swerve swerve = new Swerve();
  private IntakeRollers intakeRollers = new IntakeRollers();
  private IntakePivot intakePivot = new IntakePivot();
  private ShooterFlywheels shooterFlywheels = new ShooterFlywheels();
  private ShooterPivot shooterPivot = new ShooterPivot();
  private HandoffRollers handoffRollers = new HandoffRollers();
  private Climber climber = new Climber();

  @SuppressWarnings("unused")
  private Indications indications =
      new Indications(
          swerve,
          intakePivot,
          shooterPivot,
          intakeRollers,
          handoffRollers,
          shooterFlywheels,
          driverController);

  // Factories
  private IntakeShooter intakeShooter =
      new IntakeShooter(handoffRollers, intakePivot, intakeRollers, shooterFlywheels, shooterPivot);
  private ClimberFactory climberFactory = new ClimberFactory(climber, shooterPivot);

  // Auto Objects
  private AutoRoutines autos =
      new AutoRoutines(swerve, shooterFlywheels, shooterPivot, intakeShooter, intakePivot);
  private Command autoCommand;
  private SysIdRoutines sysIdRoutines;

  // Bind commands to triggers
  private void configureTeleopBindings() {
    // Default telop drive command
    swerve.setDefaultCommand(
        swerve.teleopDriveCommand(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> false));

    // Sets the default position to be home
    intakePivot.setDefaultCommand(intakePivot.setIntakeDown(IntakePosition.HOME));

    driverController.rightStick().onTrue(swerve.zeroGyroCommand());
    driverController.start().toggleOnTrue(swerve.xSwerveCommand());
    driverController
        .rightBumper()
        .onTrue(
            Commands.either(
                intakeShooter.shootSpeaker(),
                intakeShooter.shootAmp(),
                () -> shooterPivot.getGoalPosition() == ShooterPosition.HOME));

    driverController
        .leftBumper()
        .and(() -> !intakeRollers.hasPiece() && !handoffRollers.hasPiece())
        .whileTrue(intakeShooter.intakeProcess());

    driverController.y().onTrue(climberFactory.goUpFully());
    driverController.a().whileTrue(climber.climbDown(3).unless(() -> !climber.getEndGame()));
    driverController.x().onTrue(Commands.either(shooterPivot.goToPositionCommand(ShooterPosition.CLIMB), climberFactory.ShooterPivotToHome(), () -> shooterPivot.getGoalPosition() != ShooterPosition.CLIMB));

    driverController
        .leftTrigger()
        .and(driverController.rightTrigger())
        .and(() -> shooterPivot.getGoalPosition() == ShooterPosition.HOME)
        .onTrue(intakeShooter.pivotAmp());
  }

  private void configureCommands() {
    new Trigger(intakeRollers::hasPiece)
        .and(() -> !handoffRollers.hasPiece())
        .and(intakePivot::isHome)
        .and(DriverStation::isTeleopEnabled)
        .onTrue(intakeShooter.handOff());
  }

  // Bind commands to triggers
  private void configureSysIdBindings() {
    // Default telop drive command
    swerve.setDefaultCommand(
        swerve.teleopDriveCommand(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            driverController.getHID()::getLeftBumper));

    driverController.rightStick().onTrue(swerve.zeroGyroCommand());
    driverController.a().whileTrue(Commands.deferredProxy(sysIdRoutines::getCommand));
  }

  private void disableBrakeMode() {
    shooterPivot.setBrakeModeCommand(false).schedule();
  }

  @Override
  public void robotInit() {
    // Configure NetworkTables for simulation with photonvision running
    if (Robot.isSimulation() && Constants.simMode != SimMode.DESKTOP) {
      NetworkTableInstance instance = NetworkTableInstance.getDefault();
      instance.stopServer();
      // set the NT server if simulating this code.
      // "localhost" for desktop simulation with photonvision running, "photonvision.local" or IP
      // address for hardware in loop simulation
      if (Constants.simMode == SimMode.DESKTOP_VISION) instance.setServer("localhost");
      else instance.setServer("photonvision.local");
      instance.startClient4("myRobot");
    }

    // Start data logs
    DataLogManager.start();
    try {
      var buffer =
          new BufferedReader(
              new FileReader(new File(Filesystem.getDeployDirectory(), "gitdata.txt")));
      DataLogManager.log(buffer.readLine());
      buffer.close();
    } catch (IOException e) {
      DataLogManager.log("ERROR Can't get git data!");
    }

    DriverStation.startDataLog(DataLogManager.getLog());
    if (Robot.isSimulation()) DataLogManager.log("Simmode is " + Constants.simMode);

    // Start Monologue
    Monologue.setupMonologue(this, "system", Constants.logFileOnly, Constants.logLazy);
    Monologue.setFileOnly(DriverStation.isFMSAttached() ? true : Constants.logFileOnly);

    // Configure command bindings
    if (Constants.testMode != Constants.TestMode.SYSID) configureTeleopBindings();
    else if (Constants.testMode == Constants.TestMode.SYSID) {
      sysIdRoutines = new SysIdRoutines(Set.of(swerve));
      configureSysIdBindings();
    }

    if (Constants.testMode == Constants.TestMode.NO_BRAKE_MODE) disableBrakeMode();

    // Configure automated commands
    configureCommands();

    // Start auto selector
    autoCommand = autos.getSelector().getSelected();
    autos.getSelector().onChange((command) -> autoCommand = command);
    Shuffleboard.getTab("Auto").add("Auto selector", autos.getSelector());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Monologue.updateAll();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    intakePivot.reset();
    autoCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    autoCommand.cancel();
  }

  @Override
  public void teleopInit() {
    shooterFlywheels.setDefaultCommand(
        Commands.either(
            shooterFlywheels.otherShoot(4),
            shooterFlywheels.otherShoot(0),
            () -> shooterPivot.getGoalPosition() != ShooterPosition.AMP));
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
