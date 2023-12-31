package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kOI;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kSwerve.Auton;
import frc.robot.utilities.ChassisLimiter;
import frc.robot.utilities.MAXSwerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
  // NT Objects
  NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
  NetworkTable ntTable = ntInst.getTable("system/drivetrain");
  NetworkTable modulesTable = ntTable.getSubTable("modules");
  NetworkTable poseTable = ntTable.getSubTable("pose");

  // Hardware
  private final MAXSwerve frontLeft =
      new MAXSwerve(
          kSwerve.CANID.frontLeftDrive,
          kSwerve.CANID.frontLeftSteer,
          kSwerve.Offsets.frontLeft,
          modulesTable.getSubTable("FrontLeft"));

  private final MAXSwerve backLeft =
      new MAXSwerve(
          kSwerve.CANID.backLeftDrive,
          kSwerve.CANID.backLeftSteer,
          kSwerve.Offsets.backLeft,
          modulesTable.getSubTable("BackLeft"));

  private final MAXSwerve backRight =
      new MAXSwerve(
          kSwerve.CANID.backRightDrive,
          kSwerve.CANID.backRightSteer,
          kSwerve.Offsets.backRight,
          modulesTable.getSubTable("BackRight"));

  private final MAXSwerve frontRight =
      new MAXSwerve(
          kSwerve.CANID.frontRightDrive,
          kSwerve.CANID.frontRightSteer,
          kSwerve.Offsets.frontRight,
          modulesTable.getSubTable("FrontRight"));

  private final AHRS navX = new AHRS(kSwerve.navxPort);

  // Controls objects
  private final SwerveDrivePoseEstimator poseEstimator;
  private final ChassisLimiter limiter;
  private Rotation2d gyroOffset = new Rotation2d();
  private ChassisSpeeds chassisVelocity = new ChassisSpeeds();

  // Logging
  private final Field2d field2d = new Field2d();
  private final FieldObject2d autonRobot = field2d.getObject("Autonomous Pose");
  private final FieldObject2d autonPath = field2d.getObject("Autonomous Path");
  private final DoublePublisher rawGyroPub = ntTable.getDoubleTopic("Raw Gyro").publish();
  private final DoublePublisher offsetGyroPub = ntTable.getDoubleTopic("Offset Gyro").publish();
  private final DoubleArrayPublisher chassisVelPub =
      ntTable.getDoubleArrayTopic("Commanded Chassis Velocity").publish();
  private final DoubleArrayPublisher measuredVelPub =
      ntTable.getDoubleArrayTopic("Actual Chassis Velocity").publish();
  private final DoubleArrayPublisher swerveStatesPub =
      ntTable.getDoubleArrayTopic("Swerve Module States").publish();

  // Simulation
  private final SimDeviceSim simNavX = new SimDeviceSim("navX-Sensor", 0);
  private final SimDouble simNavXYaw = simNavX.getDouble("Yaw");

  public Swerve() {
    // Setup controls objects
    limiter = new ChassisLimiter(kSwerve.maxTransAccel, kSwerve.maxAngAccel);
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kSwerve.kinematics,
            getGyroRaw(),
            new SwerveModulePosition[] {
              frontLeft.getPositon(),
              backLeft.getPositon(),
              backRight.getPositon(),
              frontRight.getPositon()
            },
            new Pose2d(4, 4, new Rotation2d()));

    // Register the Field2d object as a sendable
    SendableBuilderImpl builder = new SendableBuilderImpl();
    builder.setTable(poseTable);
    SendableRegistry.publish(field2d, builder);
    builder.startListeners();

    // Bind Path Follower command logging methods
    PathPlannerLogging.setLogActivePathCallback(autonPath::setPoses);
    PathPlannerLogging.setLogTargetPoseCallback(autonRobot::setPose);
  }

  // ---------- Drive Commands ----------

  // Telop field oriented driver commands
  public Command teleopDriveCommand(
      DoubleSupplier xTranslation,
      DoubleSupplier yTranslation,
      DoubleSupplier zRotation,
      BooleanSupplier boost) {
    return this.run(
        () ->
            drive(
                fieldToRobotSpeeds(
                    joystickToChassis(
                        xTranslation.getAsDouble(),
                        yTranslation.getAsDouble(),
                        zRotation.getAsDouble(),
                        boost.getAsBoolean())),
                kSwerve.Teleop.closedLoop));
  }

  // Returns a command that locks onto the provided heading while translation is driver controlled
  public Command teleopLockHeadingCommand(
      DoubleSupplier xTranslation,
      DoubleSupplier yTranslation,
      Rotation2d heading,
      BooleanSupplier boost) {

    ProfiledPIDController headingController =
        new ProfiledPIDController(
            kSwerve.Auton.angP,
            0,
            kSwerve.Auton.angD,
            new Constraints(kSwerve.maxAngSpeed, kSwerve.maxAngAccel));
    headingController.enableContinuousInput(0, 2 * Math.PI);

    return this.runOnce(() -> headingController.reset(getGyro().getRadians(), getGyroYawRate()))
        .andThen(new PrintCommand("Started lock command at angle " + heading.getRadians()))
        .andThen(
            this.run(
                () -> {
                  var speeds =
                      joystickToChassis(
                          xTranslation.getAsDouble(),
                          yTranslation.getAsDouble(),
                          0,
                          boost.getAsBoolean());
                  speeds.omegaRadiansPerSecond =
                      headingController.calculate(
                          getPose().getRotation().getRadians(), heading.getRadians());
                  drive(fieldToRobotSpeeds(speeds), false);
                }));
  }

  // Returns a command that controls the heading to face a given point on the field while tranlation
  // is driver controlled
  public Command teleopFocusPointCommand(
      DoubleSupplier xTranslation,
      DoubleSupplier yTranslation,
      Translation2d point,
      BooleanSupplier boost) {

    ProfiledPIDController headingController =
        new ProfiledPIDController(
            kSwerve.Auton.angP,
            0,
            kSwerve.Auton.angD,
            new Constraints(kSwerve.maxAngSpeed, kSwerve.maxAngAccel));
    headingController.enableContinuousInput(0, 2 * Math.PI);

    return this.runOnce(() -> headingController.reset(getGyro().getRadians(), getGyroYawRate()))
        .andThen(
            this.run(
                () -> {
                  var speeds =
                      joystickToChassis(
                          xTranslation.getAsDouble(),
                          yTranslation.getAsDouble(),
                          0,
                          boost.getAsBoolean());
                  speeds.omegaRadiansPerSecond =
                      headingController.calculate(
                          getPose().getRotation().getRadians(),
                          getPose()
                              .getTranslation()
                              .minus(point)
                              .getAngle()
                              .plus(Rotation2d.fromDegrees(180))
                              .getRadians());
                  drive(fieldToRobotSpeeds(speeds), false);
                }));
  }

  // ---------- Autonomous Commands ----------

  // Follow a PathPlanner path
  public Command followPathCommand(PathPlannerPath path) {
    return new FollowPathHolonomic(
        path,
        this::getPose,
        this::getChassisSpeeds,
        (speeds) -> drive(speeds, true),
        Auton.pathFollowConfig,
        this);
  }

  // Follow a PathPlanner path and trigger commands passed in the event map at event markers
  public Command followPathWithEventsCommand(PathPlannerPath path) {
    return new FollowPathWithEvents(followPathCommand(path), path, this::getPose);
  }

  // Generate an on-the-fly path to reach a certain pose
  public Command driveToPointCommand(Pose2d goalPose) {
    return driveToPoint(goalPose, goalPose.getRotation());
  }

  // Generate an on-the-fly path to reach a certain pose with a given holonomic rotation
  public Command driveToPoint(Pose2d goalPose, Rotation2d holonomicRotation) {
    return this.defer(
        () ->
            followPathCommand(
                new PathPlannerPath(
                    PathPlannerPath.bezierFromPoses(getPose(), goalPose),
                    new PathConstraints(
                        kSwerve.Auton.maxVel,
                        kSwerve.Auton.maxAccel,
                        kSwerve.Auton.maxAngVel,
                        kSwerve.maxAngAccel),
                    new GoalEndState(0.0, holonomicRotation))));
  }

  // ---------- Other commands ----------

  // Put wheels into x configuration
  public Command xSwerveCommand() {
    return this.startEnd(this::xSwerve, () -> {});
  }

  // Zero the gyro
  public Command zeroGyroCommand() {
    return this.runOnce(this::zeroGyro);
  }

  // Reset the gyro with pose estimation (don't use without vision)
  public Command resetGyroCommand() {
    return this.runOnce(this::matchGyroToPose);
  }

  // ---------- Public interface methods ----------

  // Drive chassis-oriented (optional flag for closed loop velocity control)
  public void drive(ChassisSpeeds speeds, boolean closedLoopDrive) {
    // TODO log requested speeds
    speeds = limiter.calculate(speeds);
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    var targetStates = kSwerve.kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, kSwerve.kModule.maxWheelSpeed);

    // TODO log commanded speeds
    chassisVelocity = speeds;
    setStates(targetStates, closedLoopDrive);
  }

  // Set wheels to x configuration
  public void xSwerve() {
    frontLeft.setX();
    backLeft.setX();
    backRight.setX();
    frontRight.setX();
  }

  // Set the drive motors to brake or coast
  public void setBrakeMode(boolean on) {
    frontLeft.setBrakeMode(on);
    backLeft.setBrakeMode(on);
    backRight.setBrakeMode(on);
    frontRight.setBrakeMode(on);
  }

  // Retrieve the pose estimation pose
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // Retrieve measured ChassisSpeeds
  public ChassisSpeeds getChassisSpeeds() {
    return kSwerve.kinematics.toChassisSpeeds(getStates());
  }

  // Set an initial pose for the pose estimator
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getGyroRaw(), getPositions(), pose);
  }

  // Zero out the gyro (current heading becomes 0)
  public void zeroGyro() {
    gyroOffset = getGyroRaw();
  }

  // Reset gyro using pose estimator
  public void matchGyroToPose() {
    gyroOffset = getGyroRaw().minus(getPose().getRotation());
  }

  // Get gyro yaw rate (radians/s CCW +)
  public double getGyroYawRate() {
    return Units.degreesToRadians(-navX.getRate());
  }

  // ---------- Private hardware interface methods ----------

  // Get direct gyro reading as Rotation2d
  private Rotation2d getGyroRaw() {
    return navX.getRotation2d();
  }

  // Get software offset gyro angle
  private Rotation2d getGyro() {
    return getGyroRaw().minus(gyroOffset);
  }

  // Retrieve the positions (angle and distance traveled) for each swerve module
  private SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPositon(), backLeft.getPositon(), backRight.getPositon(), frontRight.getPositon()
    };
  }

  // Retrieve the state (velocity and heading) for each swerve module
  private SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), backLeft.getState(), backRight.getState(), frontRight.getState()
    };
  }

  // Set desired states (angle and velocity) to each module with an optional flag to enable closed
  // loop control on velocity
  private void setStates(SwerveModuleState[] states, boolean closedLoopDrive) {
    frontLeft.setTargetState(states[0], closedLoopDrive);
    backLeft.setTargetState(states[1], closedLoopDrive);
    backRight.setTargetState(states[2], closedLoopDrive);
    frontRight.setTargetState(states[3], closedLoopDrive);
  }

  // ---------- Periodic ----------

  // Update pose estimator and log data
  @Override
  public void periodic() {
    if (RobotBase.isSimulation())
      simNavXYaw.set(
          simNavXYaw.get() + chassisVelocity.omegaRadiansPerSecond * -360 / (2 * Math.PI) * 0.02);
    poseEstimator.update(getGyroRaw(), getPositions());
    poseEstimator.addVisionMeasurement(getPose(), getGyroYawRate());
    log();
  }

  // ---------- Helpers ----------

  private ChassisSpeeds joystickToChassis(
      double xTranslation, double yTranslation, double zRotation, boolean boost) {

    // Apply deadzones
    if (Math.abs(xTranslation) <= kOI.translationDeadzone) xTranslation = 0;
    if (Math.abs(yTranslation) <= kOI.translationDeadzone) yTranslation = 0;
    if (Math.abs(zRotation) <= kOI.rotationDeadzone) zRotation = 0;

    // Square inputs for controlabitly
    xTranslation = Math.copySign(xTranslation * xTranslation, xTranslation);
    yTranslation = Math.copySign(yTranslation * yTranslation, yTranslation);
    zRotation = Math.copySign(zRotation * zRotation, zRotation);

    // Create a velocity vector (full speed is a unit vector)
    var translationVelocity = VecBuilder.fill(xTranslation, yTranslation);

    // Multiply velocity vector by max speed
    translationVelocity = translationVelocity.times(kSwerve.maxTransSpeed);

    // Contrain velocities to boost gain
    if (!boost) translationVelocity = translationVelocity.times(kSwerve.Teleop.translationGain);
    zRotation *= kSwerve.maxAngSpeed * kSwerve.Teleop.rotationGain;

    // Construct chassis speeds and return
    return new ChassisSpeeds(
        translationVelocity.get(0, 0), translationVelocity.get(1, 0), zRotation);
  }

  // Save a few characters calling the full thing
  private ChassisSpeeds fieldToRobotSpeeds(ChassisSpeeds speeds) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyro());
  }

  // Log data to network tables
  private void log() {
    frontLeft.updateNT();
    backLeft.updateNT();
    backRight.updateNT();
    frontRight.updateNT();

    rawGyroPub.set(getGyroRaw().getRadians());
    offsetGyroPub.set(getGyro().getRadians());
    // Send the chassis velocity as a double array (vel_x, vel_y, omega_z)
    chassisVelPub.set(
        new double[] {
          chassisVelocity.vxMetersPerSecond,
          chassisVelocity.vyMetersPerSecond,
          chassisVelocity.omegaRadiansPerSecond
        });
    var measuredVel =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            kSwerve.kinematics.toChassisSpeeds(getStates()), getGyro().unaryMinus());
    measuredVelPub.set(
        new double[] {
          measuredVel.vxMetersPerSecond, measuredVel.vyMetersPerSecond, getGyroYawRate()
        });

    swerveStatesPub.set(
        new double[] {
          frontLeft.getTargetState().angle.getRadians(),
              frontLeft.getTargetState().speedMetersPerSecond,
          backLeft.getTargetState().angle.getRadians(),
              backLeft.getTargetState().speedMetersPerSecond,
          backRight.getTargetState().angle.getRadians(),
              backRight.getTargetState().speedMetersPerSecond,
          frontRight.getTargetState().angle.getRadians(),
              frontRight.getTargetState().speedMetersPerSecond
        });

    field2d.setRobotPose(getPose());
    autonRobot.setPose(new Pose2d(8, 4, Rotation2d.fromDegrees(90)));
  }
}
