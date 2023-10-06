// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class Robot extends TimedRobot {
  private PS4Controller driverController;
  private System system = new System();
  private Command autoCommand = null;

  private void configureBindings() {
    system.configureTeleopDrive(
        driverController::getLeftX,
        driverController::getLeftY,
        driverController::getRightX,
        driverController::getR2Button);
  }

  @Override
  public void robotInit() {
    // Configure NetworkTables for simulation with photonvision running
    if (Robot.isSimulation()) {
      NetworkTableInstance instance = NetworkTableInstance.getDefault();
      instance.stopServer();
      // set the NT server if simulating this code.
      // "localhost" for photon on desktop, or "photonvision.local" / "[ip-address]" for coprocessor
      instance.setServer("localhost");
      instance.startClient4("myRobot");
    }

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
    configureBindings();
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
    autoCommand = system.getAuto();
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
