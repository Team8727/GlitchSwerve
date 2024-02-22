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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SimMode;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.SysIdRoutines;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.ShooterFlywheels;
import frc.robot.subsystems.Swerve;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import monologue.Logged;
import monologue.Monologue;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot implements Logged {
  private CommandXboxController driverController = new CommandXboxController(0);

  //Subsystems
  private Swerve swerve = new Swerve();
  private IntakeRollers intakeRollers = new IntakeRollers();
  private IntakePivot intakePivot = new IntakePivot();
  private ShooterFlywheels shooterFlywheels = new ShooterFlywheels();
  
  //Auto Objects
  private AutoRoutines autos = new AutoRoutines(swerve);
  private Command autoCommand = null;

  //SysId Objects
  private SysIdRoutines sysIdRoutines = new SysIdRoutines(swerve, intakePivot);

  // Bind commands to triggers
  private void configureBindings() {
    // Default telop drive command
    swerve.setDefaultCommand(
        swerve.teleopDriveCommand(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            driverController.getHID()::getLeftBumper));
    
    //Sets the default position to be home
    intakePivot.setDefaultCommand(intakePivot.setIntakeDown(false));

    driverController.rightStick().onTrue(swerve.zeroGyroCommand());
    driverController.start().toggleOnTrue(swerve.xSwerveCommand());
    /* 
    driverController
        .a()
        .whileTrue(Commands.deferredProxy(() -> sysIdRoutines.getSelector().getSelected()));*/
    
    driverController.b().whileTrue(intakePivot.setIntakeDown(true).alongWith(intakeRollers.intakeCommand()));
    
  }

  private void configureCommands(){
    new Trigger(intakeRollers::getPieceCheck).and(() -> !shooterFlywheels.getPieceCheck()).onTrue(intakeRollers.outtakeCommand().alongWith(shooterFlywheels.intakeCommand()));
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
    configureBindings();

    // Configure automated commands
    configureCommands();

    // Start auto selector
    autoCommand = autos.getSelector().getSelected();
    autos.getSelector().onChange((command) -> autoCommand = command);
    Shuffleboard.getTab("Auto").add("Auto selector", autos.getSelector());

    // Start sysId selector
    Shuffleboard.getTab("SysId").add("SysID selector", sysIdRoutines.getSelector());
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
    autoCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    autoCommand.cancel();
  }

  @Override
  public void teleopInit() {}

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
