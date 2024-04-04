package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.kOI;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kSwerve.Auton;
import frc.robot.Constants.kSwerve.kModule;
import frc.robot.Constants.kSwerveShoot;
import frc.robot.commands.SysIdRoutines.SysIdType;
import frc.robot.utilities.Characterizable;
import frc.robot.utilities.ChassisLimiter;
import frc.robot.utilities.MAXSwerve;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Swerve extends SubsystemBase implements Logged, Characterizable {
  // Hardware
  private final MAXSwerve frontLeftModule =
      new MAXSwerve(
          kSwerve.CANID.frontLeftDrive, kSwerve.CANID.frontLeftSteer, kSwerve.Offsets.frontLeft);

  private final MAXSwerve backLeftModule =
      new MAXSwerve(
          kSwerve.CANID.backLeftDrive, kSwerve.CANID.backLeftSteer, kSwerve.Offsets.backLeft);

  private final MAXSwerve backRightModule =
      new MAXSwerve(
          kSwerve.CANID.backRightDrive, kSwerve.CANID.backRightSteer, kSwerve.Offsets.backRight);

  private final MAXSwerve frontRightModule =
      new MAXSwerve(
          kSwerve.CANID.frontRightDrive, kSwerve.CANID.frontRightSteer, kSwerve.Offsets.frontRight);

  private final AHRS navX = new AHRS(kSwerve.navxPort);

  // Controls objects
  private final SwerveDrivePoseEstimator poseEstimator;
  private final ChassisLimiter limiter;
  @Log.NT private ChassisSpeeds chassisVelocityTarget = new ChassisSpeeds();

  public class SwerveState {
    public enum Mode {
      DRIVE,
      HEADING_LOCK,
      POINT_OF_INTEREST,
      AUTO_DRIVE,
      PARK,
      IDLE
    }

    public Mode mode;
    public boolean boosting;

    public SwerveState() {
      mode = Mode.IDLE;
      boosting = false;
    }

    public void set(Mode mode, boolean boosting) {
      this.mode = mode;
      this.boosting = boosting;
    }
  }

  private SwerveState swerveState = new SwerveState();

  // Logging
  @Log.NT private final Field2d field2d = new Field2d();
  private final FieldObject2d autonRobot = field2d.getObject("Autonomous Pose");
  private final FieldObject2d autonPath = field2d.getObject("Autonomous Path");

  // Simulation
  private final SimDeviceSim simNavX = new SimDeviceSim("navX-Sensor", 0);
  private final SimDouble simNavXYaw = simNavX.getDouble("Yaw");

  // Vision Objects
  private PhotonCamera camera1 = new PhotonCamera("camera1");
  private PhotonCamera camera2 = new PhotonCamera("camera2");
  private AprilTagFieldLayout fieldLayout;
  private PhotonPoseEstimator photonPoseEstimator1;
  private PhotonPoseEstimator photonPoseEstimator2;
  @Log.NT private Pose3d photonPose = new Pose3d();

  private boolean visionEnable = false;

  // Path following
  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private ProfiledPIDController rotController;
  private TrapezoidProfile.Constraints pathFollowConstraints =
      new Constraints(Auton.maxOnTheFlyVel, Auton.maxOnTheFlyAcc);
  private ChassisSpeeds pathChassisSpeeds;

  public Swerve() {
    Shuffleboard.getTab("Swerve").add(this);
    // Setup controls objects
    limiter = new ChassisLimiter(kSwerve.maxTransAccel, kSwerve.maxAngAccel);
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kSwerve.kinematics,
            getGyroRaw(),
            new SwerveModulePosition[] {
              frontLeftModule.getPositon(),
              backLeftModule.getPositon(),
              backRightModule.getPositon(),
              frontRightModule.getPositon()
            },
            new Pose2d(0, 0, new Rotation2d()),
            kSwerve.stateStdDevs,
            kSwerve.visionStdDevs);

    // Vision
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (Exception e) {
      System.out.println("Failed to load field layout");
    }
    photonPoseEstimator1 =
        new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera1,
            kSwerve.aprilTagCamera1PositionTransform);
    photonPoseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    photonPoseEstimator2 =
        new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera2,
            kSwerve.aprilTagCamera2PositionTransform);
    photonPoseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // Bind Path Follower command logging methods
    PathPlannerLogging.setLogActivePathCallback(autonPath::setPoses);
    PathPlannerLogging.setLogTargetPoseCallback(autonRobot::setPose);

    // Path Follower Profiles
    xController = new ProfiledPIDController(Auton.transP, 0, 0, pathFollowConstraints);
    xController.setTolerance(0.02);
    yController = new ProfiledPIDController(Auton.transP, 0, 0, pathFollowConstraints);
    yController.setTolerance(0.02);
    rotController = new ProfiledPIDController(Auton.angP, 0, 0, pathFollowConstraints);
    rotController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // ---------- Drive Commands ----------

  // Telop field oriented driver commands
  public Command teleopDriveCommand(
      DoubleSupplier xTranslation,
      DoubleSupplier yTranslation,
      DoubleSupplier zRotation,
      BooleanSupplier boost) {
    return this.run(
            () -> {
              drive(
                  driverToChassisSpeeds(
                      joystickToChassis(
                          xTranslation.getAsDouble(),
                          yTranslation.getAsDouble(),
                          zRotation.getAsDouble(),
                          boost.getAsBoolean())),
                  kSwerve.Teleop.closedLoop);
              swerveState.set(SwerveState.Mode.DRIVE, boost.getAsBoolean());
            })
        .finallyDo(() -> swerveState.set(SwerveState.Mode.IDLE, false))
        .withName("telopDriveCommand");
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

    return this.runOnce(() -> headingController.reset(getHeading().getRadians(), getGyroYawRate()))
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
                  drive(driverToChassisSpeeds(speeds), false);
                  swerveState.set(SwerveState.Mode.HEADING_LOCK, boost.getAsBoolean());
                }))
        .finallyDo(() -> swerveState.set(SwerveState.Mode.IDLE, false))
        .withName("teleopLockHeadingCommand");
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

    return this.runOnce(() -> headingController.reset(getGyroRaw().getRadians(), getGyroYawRate()))
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
                  drive(driverToChassisSpeeds(speeds), false);
                  swerveState.set(SwerveState.Mode.POINT_OF_INTEREST, boost.getAsBoolean());
                }))
        .finallyDo(() -> swerveState.set(SwerveState.Mode.IDLE, false))
        .withName("teleopFocusPointCommand");
  }

  // ---------- Autonomous Commands ----------

  // Follow a PathPlanner path
  public Command followPathCommand(PathPlannerPath path, boolean useAlliance, boolean resetPose) {
    return this.runOnce(() -> swerveState.mode = SwerveState.Mode.AUTO_DRIVE)
        .andThen(
            () -> {
              if (resetPose) resetPose(path, useAlliance);
            })
        // working
        .andThen(
            new FollowPathHolonomic(
                path,
                this::getPose,
                this::getChassisSpeeds,
                (speeds) -> drive(speeds, true),
                Auton.pathFollowConfig,
                () -> {
                  if (useAlliance && DriverStation.getAlliance().isPresent()) {
                    if (DriverStation.getAlliance().get() == Alliance.Red) return true;
                  }
                  return false;
                },
                this))
        .finallyDo(() -> swerveState.mode = SwerveState.Mode.IDLE)
        .withName("followPathCommand");
  }

  public Command followPathCommand(PathPlannerPath path, boolean useAlliance) {
    return followPathCommand(path, useAlliance, false);
  }

  // Generate an on-the-fly path to reach a certain pose
  public Command driveToPointCommand(Pose2d goalPose) {
    return driveToPoint(goalPose, goalPose.getRotation()).withName("driveToPointCommand");
  }

  // Generate an on-the-fly path to reach a certain pose with a given holonomic rotation
  public Command driveToPoint(Pose2d goalPose, Rotation2d holonomicRotation) {
    return this.runOnce(() -> swerveState.mode = SwerveState.Mode.AUTO_DRIVE)
        .andThen(
            this.defer(
                () ->
                    followPathCommand(
                        new PathPlannerPath(
                            PathPlannerPath.bezierFromPoses(getPose(), goalPose),
                            new PathConstraints(
                                kSwerve.Auton.maxVel,
                                kSwerve.Auton.maxAccel,
                                kSwerve.Auton.maxAngVel,
                                kSwerve.maxAngAccel),
                            new GoalEndState(0.0, holonomicRotation)),
                        false)))
        .finallyDo(() -> swerveState.mode = SwerveState.Mode.IDLE)
        .withName("driveToPoint");
  }

  public Command driveToPointProfiles(Pose2d goal) {
    return this.runOnce(
            () -> {
              // Reset Profiles
              ChassisSpeeds speeds =
                  ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getHeading());
              xController.reset(getPose().getX(), speeds.vxMetersPerSecond);
              yController.reset(getPose().getY(), speeds.vyMetersPerSecond);
              rotController.reset(
                  getPose().getRotation().getRadians(), speeds.omegaRadiansPerSecond);

              // Set Goals
              xController.setGoal(goal.getX());
              yController.setGoal(goal.getY());
              rotController.setGoal(goal.getRotation().getRadians());
            })
        .andThen(
            this.run(
                    () -> {
                      // Calculate Feedback
                      double xFB = xController.calculate(getPose().getX());
                      double yFB = yController.calculate(getPose().getY());

                      double rotFB = rotController.calculate(getPose().getRotation().getRadians());

                      // Calculate full chassis Speeds
                      pathChassisSpeeds =
                          new ChassisSpeeds(
                              xFB + xController.getSetpoint().velocity,
                              yFB + yController.getSetpoint().velocity,
                              rotFB + rotController.getSetpoint().velocity);
                      this.log("xControllerSetpoint", xController.getSetpoint().position);
                      this.log("yControllerSetpoint", yController.getSetpoint().position);
                      this.log("rotController", rotController.getSetpoint().position);
                      this.log("xSetVel", xController.getSetpoint().velocity);
                      this.log("ySetVel", yController.getSetpoint().velocity);
                      this.log("rotSetVel", rotController.getSetpoint().velocity);
                      this.log("xFB", xFB);
                      this.log("yFB", yFB);
                      this.log("rotFB", rotFB);
                      drive(
                          ChassisSpeeds.fromFieldRelativeSpeeds(pathChassisSpeeds, getHeading()),
                          true);
                    })
                .until(
                    () -> xController.atGoal() && yController.atGoal() && rotController.atGoal()));
  }

  public Command driveFieldSpeedsCommand(ChassisSpeeds fieldRelativeSpeeds) {
    return this.startEnd(
        () ->
            drive(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getHeading()), false),
        () -> drive(new ChassisSpeeds(), false));
  }

  // ---------- Other commands ----------

  // Put wheels into x configuration
  public Command xSwerveCommand() {
    return this.runOnce(() -> swerveState.mode = SwerveState.Mode.PARK)
        .andThen(this.startEnd(this::xSwerve, () -> {}))
        .finallyDo(() -> swerveState.mode = SwerveState.Mode.IDLE)
        .withName("xSwerveCommand");
  }

  // Zero the gyro
  public Command zeroGyroCommand() {
    return Commands.runOnce(() -> zeroGyro()).withName("zeroGyroCommand");
  }

  // ---------- SysId Commands ----------
  public SysIdRoutine getRoutine(SysIdType routineType) {
    MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));

    if (routineType == SysIdType.ANGULAR) {
      MutableMeasure<Angle> angle = mutable(Radians.of(0));
      MutableMeasure<Velocity<Angle>> angularVelocity = mutable(RadiansPerSecond.of(0));
      return new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (volts) -> setStateSysId(volts, routineType),
              (log) -> {
                log.motor("Chassis")
                    .voltage(
                        appliedVoltage.mut_replace(frontLeftModule.getRawDriveNeoVoltage(), Volts))
                    .angularPosition(angle.mut_replace(getGyroRaw().getRadians(), Radians))
                    .angularVelocity(
                        angularVelocity.mut_replace(getGyroYawRate(), RadiansPerSecond));
              },
              this));
    } else {
      MutableMeasure<Distance> distance = mutable(Meters.of(0));
      MutableMeasure<Velocity<Distance>> velocity = mutable(MetersPerSecond.of(0));
      return new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (volts) -> setStateSysId(volts, routineType),
              (log) -> {
                logModuleSysId(log, frontLeftModule, appliedVoltage, distance, velocity);
                logModuleSysId(log, backLeftModule, appliedVoltage, distance, velocity);
                logModuleSysId(log, backRightModule, appliedVoltage, distance, velocity);
                logModuleSysId(log, frontRightModule, appliedVoltage, distance, velocity);
              },
              this));
    }
  }

  // ---------- Public interface methods ----------

  // Drive chassis-oriented (optional flag for closed loop velocity control)
  public void drive(ChassisSpeeds speeds, boolean closedLoopDrive) {
    // Record targeted speed
    chassisVelocityTarget = speeds;
    limiter.update(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getHeading()));

    // Discretize to reduce drift when rotating
    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // Convert to module states and desaturate speeds to prevent exceeding module capabilities
    var targetStates = kSwerve.kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, kModule.maxWheelSpeed);
    speeds = kSwerve.kinematics.toChassisSpeeds(targetStates);

    setStates(targetStates, closedLoopDrive);
  }

  // Convert driver field relative speeds to chassis speeds
  public ChassisSpeeds driverToChassisSpeeds(ChassisSpeeds speeds) {
    speeds = limiter.calculate(speeds);
    return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
  }

  // Set wheels to x configuration
  public void xSwerve() {
    frontLeftModule.setX();
    backLeftModule.setX();
    backRightModule.setX();
    frontRightModule.setX();
  }

  // Set the drive motors to brake or coast
  public void setBrakeMode(boolean on) {
    frontLeftModule.setBrakeMode(on);
    backLeftModule.setBrakeMode(on);
    backRightModule.setBrakeMode(on);
    frontRightModule.setBrakeMode(on);
  }

  // Retrieve the pose estimation pose
  @Log.NT
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // Retrieve measured ChassisSpeeds
  @Log.NT
  public ChassisSpeeds getChassisSpeeds() {
    return kSwerve.kinematics.toChassisSpeeds(getModuleStates());
  }

  @Log.NT
  public boolean getVisionEnable() {
    return visionEnable;
  }

  public void setVisionEnable(boolean bool) {
    visionEnable = bool;
  }
  // AddVisionMeasurement With Two camera streams
  public void updatePoseWithCameraData() {
    if (camera1.isConnected()) {
      var result = camera1.getLatestResult();
      Optional<EstimatedRobotPose> estimatedGlobalPose = photonPoseEstimator1.update(result);
      if (estimatedGlobalPose.isPresent()) {
        double distance = result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
        poseEstimator.addVisionMeasurement(
            estimatedGlobalPose.get().estimatedPose.toPose2d(),
            estimatedGlobalPose.get().timestampSeconds,
            kSwerve.visionStdDevs.times(distance * kSwerve.visionScalingFactor));
        photonPose = estimatedGlobalPose.get().estimatedPose;
      }
    }
    if (camera2.isConnected()) {
      var cam2Result = camera2.getLatestResult();
      if (cam2Result.getMultiTagResult().estimatedPose.isPresent) { // checks the pose exists
        double poseAmbiguity = cam2Result.getBestTarget().getPoseAmbiguity();
        if (poseAmbiguity < 0.2
            && poseAmbiguity >= 0) { // check if the ambiguity is in the correct bounds
          Optional<EstimatedRobotPose> estimatedGlobalPoseVision = photonPoseEstimator2.update();
          poseEstimator.addVisionMeasurement(
              estimatedGlobalPoseVision.get().estimatedPose.toPose2d(),
              estimatedGlobalPoseVision.get().timestampSeconds);
        }
      }
    }
  }

  // Set an initial pose for the pose estimator
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getGyroRaw(), getPositions(), pose);
  }

  // Zero out the gyro (current heading becomes 0)
  public void zeroGyro() {
    poseEstimator.resetPosition(
        getGyroRaw(),
        getPositions(),
        new Pose2d(getPose().getTranslation(), Rotation2d.fromRadians(0)));
  }

  // Return the heading of the robot as measured by the pose estimator
  @Log.NT
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  // Get gyro yaw rate (radians/s CCW +)
  @Log.NT
  public double getGyroYawRate() {
    return Units.degreesToRadians(navX.getRawGyroZ());
  }

  public SwerveState getSwerveState() {
    return swerveState;
  }

  public boolean isInSpeakerRange() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue)
      return kSwerveShoot.blueSpeaker.minus(getPose().getTranslation()).getNorm()
          < kSwerveShoot.spinupDistance;
    else
      return kSwerveShoot.redSpeaker.minus(getPose().getTranslation()).getNorm()
          < kSwerveShoot.spinupDistance;
  }

  // ---------- Private hardware interface methods ----------

  // Get direct gyro reading as Rotation2d
  @Log.NT
  private Rotation2d getGyroRaw() {
    return navX.getRotation2d();
  }

  // Retrieve the positions (angle and distance traveled) for each swerve module
  private SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      frontLeftModule.getPositon(),
      backLeftModule.getPositon(),
      backRightModule.getPositon(),
      frontRightModule.getPositon()
    };
  }

  // Retrieve the state (velocity and heading) for each swerve module
  @Log.NT
  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeftModule.getState(),
      backLeftModule.getState(),
      backRightModule.getState(),
      frontRightModule.getState()
    };
  }

  // Set desired states (angle and velocity) to each module with an optional flag to enable closed
  // loop control on velocity
  private void setStates(SwerveModuleState[] states, boolean closedLoopDrive) {
    frontLeftModule.setTargetState(states[0], closedLoopDrive);
    backLeftModule.setTargetState(states[1], closedLoopDrive);
    backRightModule.setTargetState(states[2], closedLoopDrive);
    frontRightModule.setTargetState(states[3], closedLoopDrive);
  }

  // ---------- Periodic ----------

  // Update pose estimator and log data
  @Override
  public void periodic() {
    if (RobotBase.isSimulation())
      simNavXYaw.set(
          simNavXYaw.get()
              + chassisVelocityTarget.omegaRadiansPerSecond * -360 / (2 * Math.PI) * 0.02);
    poseEstimator.update(getGyroRaw(), getPositions());
    if (DriverStation.isTeleop()) {
      updatePoseWithCameraData();
    }
    field2d.setRobotPose(getPose());
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

    // If we're red alliance invert translation directions because driver is rotated 180 degrees
    // from the blue origin reference
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red)
      translationVelocity = translationVelocity.times(-1);

    // Multiply velocity vector by max speed
    translationVelocity = translationVelocity.times(kSwerve.maxTransSpeed);

    // Contrain velocities to boost gain
    if (!boost) translationVelocity = translationVelocity.times(kSwerve.Teleop.translationGain);
    zRotation *= kSwerve.maxAngSpeed * kSwerve.Teleop.rotationGain;

    // Construct chassis speeds and return
    return new ChassisSpeeds(
        translationVelocity.get(0, 0), translationVelocity.get(1, 0), zRotation);
  }

  private void resetPose(PathPlannerPath path, boolean useAlliance) {
    var trajectory = path.getTrajectory(new ChassisSpeeds(), new Rotation2d());
    var initialPose = trajectory.getInitialTargetHolonomicPose();
    if (useAlliance
        && DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      setPose(GeometryUtil.flipFieldPose(initialPose));
    } else setPose(initialPose);
  }

  private void setStateSysId(Measure<Voltage> volts, SysIdType type) {
    // Set all the wheels to one direction
    if (type == SysIdType.ANGULAR) {
      frontLeftModule.setO();
      backLeftModule.setO();
      frontRightModule.setO();
      backRightModule.setO();
    } else {
      var direction =
          (type == SysIdType.LINEAR) ? Rotation2d.fromDegrees(45) : Rotation2d.fromDegrees(-45);
      frontLeftModule.setTargetState(new SwerveModuleState(0, direction), false, false);
      backLeftModule.setTargetState(new SwerveModuleState(0, direction), false, false);
      frontRightModule.setTargetState(new SwerveModuleState(0, direction), false, false);
      backRightModule.setTargetState(new SwerveModuleState(0, direction), false, false);
    }

    // apply the voltage
    frontLeftModule.setRawDriveVoltage(volts.magnitude());
    backLeftModule.setRawDriveVoltage(volts.magnitude());
    frontRightModule.setRawDriveVoltage(volts.magnitude());
    backRightModule.setRawDriveVoltage(volts.magnitude());
  }

  private void logModuleSysId(
      SysIdRoutineLog log,
      MAXSwerve module,
      MutableMeasure<Voltage> appliedVoltage,
      MutableMeasure<Distance> distance,
      MutableMeasure<Velocity<Distance>> velocity) {
    log.motor(module.getFullPath())
        .voltage(appliedVoltage.mut_replace(module.getRawDriveNeoVoltage(), Volts))
        .linearPosition(distance.mut_replace(module.getPositon().distanceMeters, Meters))
        .linearVelocity(
            velocity.mut_replace(module.getState().speedMetersPerSecond, MetersPerSecond));
  }
}
