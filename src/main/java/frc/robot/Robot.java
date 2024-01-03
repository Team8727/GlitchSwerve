// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.subsystems.Swerve;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class Robot extends TimedRobot {
  private CommandXboxController driverController = new CommandXboxController(0);
  private Swerve swerve = new Swerve();
  private AutoRoutines autos = new AutoRoutines(swerve);
  private Command autoCommand = null;

  // Bind commands to triggers
  private void configureBindings() {
    // Default telop drive command
    swerve.setDefaultCommand(
        swerve.teleopDriveCommand(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            driverController.getHID()::getLeftBumper));

    // Bind a heading lock command for the four cardinal directions on the d-hat
    int[] povDirections = new int[] {0, 90, 180, 270};
    for (int direction : povDirections) {
      driverController
          .pov(direction)
          .onTrue(
              swerve
                  .teleopLockHeadingCommand(
                      () -> -driverController.getLeftY(),
                      () -> -driverController.getLeftX(),
                      Rotation2d.fromDegrees(-direction),
                      driverController.getHID()::getLeftBumper)
                  .until(() -> Math.abs(driverController.getRightX()) > 0.2));
    }

    // Chassis Faces point of interest while driver controls translation
    driverController
        .a()
        .onTrue(
            swerve
                .teleopFocusPointCommand(
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    new Translation2d(8, 4),
                    driverController.getHID()::getLeftBumper)
                .until(() -> Math.abs(driverController.getRightX()) > 0.2));

    // Automatically drive to a point on the field
    driverController
        .y()
        .onTrue(swerve.driveToPoint(new Pose2d(4, 4, Rotation2d.fromDegrees(0)), new Rotation2d()));

    driverController.rightStick().onTrue(swerve.zeroGyroCommand());
    driverController.start().toggleOnTrue(swerve.xSwerveCommand());
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

    // Configure command bindings
    configureBindings();

    // Start auto selector
    autoCommand = autos.getSelector().getSelected();
    autos.getSelector().onChange((command) -> autoCommand = command);
    Shuffleboard.getTab("Auto").add("Auto selector", autos.getSelector());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
