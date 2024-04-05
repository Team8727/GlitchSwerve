package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kSwerveShoot;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;

public class SwerveShoot {

  private Swerve swerve;
  private IntakeShooter intakeShooter;

  public SwerveShoot(Swerve swerve, IntakeShooter intakeShooter) {
    this.swerve = swerve;
    this.intakeShooter = intakeShooter;
  }

  public Command autoAmp() {
    return Commands.either(
            swerve.driveToPointProfiles(
                new Pose2d(
                    kSwerveShoot
                        .blueAmp
                        .minus(kSwerveShoot.chassisOffset)
                        .minus(kSwerveShoot.stopDistance),
                    kSwerveShoot.rotationAmpShot)),
            swerve.driveToPointProfiles(
                new Pose2d(
                    kSwerveShoot
                        .redAmp
                        .minus(kSwerveShoot.chassisOffset)
                        .minus(kSwerveShoot.stopDistance),
                    kSwerveShoot.rotationAmpShot)),
            () ->
                (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Blue))
        .andThen(
            swerve
                .driveFieldSpeedsCommand(new ChassisSpeeds(0, kSwerveShoot.bumpUpSpeed, 0))
                .withTimeout(kSwerveShoot.bumpUpTime))
        .alongWith(Commands.idle().until(swerve::isInAmpRange).andThen(intakeShooter.pivotAmp()))
        .andThen(intakeShooter.shootAmp());
  }

  public Command distanceShot(DoubleSupplier xTranslation, DoubleSupplier yTranslation) {
    return Commands.either(
        swerve
            .teleopFocusPointCommand(
                xTranslation, yTranslation, kSwerveShoot.blueSpeaker, () -> false)
            .alongWith(
                intakeShooter.angleShooterBasedOnDistance(
                    new Translation2d(swerve.getPose().getX(), swerve.getPose().getY())
                        .getDistance(kSwerveShoot.blueSpeaker))),
        swerve
            .teleopFocusPointCommand(
                xTranslation, yTranslation, kSwerveShoot.redSpeaker, () -> false)
            .alongWith(
                intakeShooter.angleShooterBasedOnDistance(
                    new Translation2d(swerve.getPose().getX(), swerve.getPose().getY())
                        .getDistance(kSwerveShoot.redSpeaker))),
        () ->
            (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Blue));
  }
}
