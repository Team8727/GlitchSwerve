package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.kOI;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kSwerve.Auton;
import frc.robot.Constants.kSwerve.kModule;
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

public class Swerve extends SubsystemBase implements Logged {
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
  @Log.File private Rotation2d gyroOffset = new Rotation2d();
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

  // SysId Objects
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine linearRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (volts) -> {
                // Set all the wheels to one direction
                frontLeftModule.setTargetState(new SwerveModuleState(0, new Rotation2d(0)), false);
                backLeftModule.setTargetState(new SwerveModuleState(0, new Rotation2d(0)), false);
                frontRightModule.setTargetState(new SwerveModuleState(0, new Rotation2d(0)), false);
                backRightModule.setTargetState(new SwerveModuleState(0, new Rotation2d(0)), false);
                // apply the voltage
                frontLeftModule.setRawDriveVoltage(volts.magnitude());
                backLeftModule.setRawDriveVoltage(volts.magnitude());
                frontRightModule.setRawDriveVoltage(volts.magnitude());
                backRightModule.setRawDriveVoltage(volts.magnitude());
              },
              (log) -> {
                log.motor("frontLeft")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontLeftModule.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(frontLeftModule.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            frontLeftModule.getState().speedMetersPerSecond, MetersPerSecond));
                log.motor("backLeft")
                    .voltage(
                        m_appliedVoltage.mut_replace(backLeftModule.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(backLeftModule.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            backLeftModule.getState().speedMetersPerSecond, MetersPerSecond));
                log.motor("frontRight")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontRightModule.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(
                            frontRightModule.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            frontRightModule.getState().speedMetersPerSecond, MetersPerSecond));
                log.motor("backRight")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            backRightModule.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(backRightModule.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            backRightModule.getState().speedMetersPerSecond, MetersPerSecond));
              },
              this));

  private final SysIdRoutine angularRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (volts) -> {
                // Set all the wheels to one direction
                frontLeftModule.setO();
                backLeftModule.setO();
                frontRightModule.setO();
                backRightModule.setO();
                // apply the voltage
                frontLeftModule.setRawDriveVoltage(volts.magnitude());
                backLeftModule.setRawDriveVoltage(volts.magnitude());
                frontRightModule.setRawDriveVoltage(volts.magnitude());
                backRightModule.setRawDriveVoltage(volts.magnitude());
              },
              (log) -> {
                log.motor("frontLeft")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontLeftModule.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(frontLeftModule.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            frontLeftModule.getState().speedMetersPerSecond, MetersPerSecond));
                log.motor("backLeft")
                    .voltage(
                        m_appliedVoltage.mut_replace(backLeftModule.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(backLeftModule.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            backLeftModule.getState().speedMetersPerSecond, MetersPerSecond));
                log.motor("frontRight")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontRightModule.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(
                            frontRightModule.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            frontRightModule.getState().speedMetersPerSecond, MetersPerSecond));
                log.motor("backRight")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            backRightModule.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(backRightModule.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            backRightModule.getState().speedMetersPerSecond, MetersPerSecond));
              },
              this));

  public Swerve() {
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
            new Pose2d(4, 4, new Rotation2d()));

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
    photonPoseEstimator2 =
        new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera2,
            kSwerve.aprilTagCamera1PositionTransform);

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
                  drive(driverToChassisSpeeds(speeds), false);
                  swerveState.set(SwerveState.Mode.POINT_OF_INTEREST, boost.getAsBoolean());
                }))
        .finallyDo(() -> swerveState.set(SwerveState.Mode.IDLE, false))
        .withName("teleopFocusPointCommand");
  }

  // ---------- Autonomous Commands ----------

  // Follow a PathPlanner path
  public Command followPathCommand(PathPlannerPath path, boolean useAlliance) {
    return this.runOnce(() -> swerveState.mode = SwerveState.Mode.AUTO_DRIVE)
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

  // Follow a PathPlanner path and trigger commands passed in the event map at event markers
  public Command followPathWithEventsCommand(PathPlannerPath path) {
    return this.runOnce(() -> swerveState.mode = SwerveState.Mode.AUTO_DRIVE)
        .andThen(followPathCommand(path, false))
        .finallyDo(() -> swerveState.mode = SwerveState.Mode.IDLE)
        .withName("followPathWithEventsCommand");
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
    return Commands.runOnce(this::zeroGyro).withName("zeroGyroCommand");
  }

  // Reset the gyro with pose estimation (don't use without vision)
  public Command resetGyroCommand() {
    return Commands.runOnce(this::matchGyroToPose).withName("resetGyroCommand");
  }

  // ---------- SysId Commands ----------
  public SysIdRoutine getLinearRoutine() {
    return linearRoutine;
  }

  public SysIdRoutine getAngularRoutine() {
    return angularRoutine;
  }

  // ---------- Public interface methods ----------

  // Drive chassis-oriented (optional flag for closed loop velocity control)
  public void drive(ChassisSpeeds speeds, boolean closedLoopDrive) {
    this.log("commandedSpeeds", speeds);

    // Record targeted speed
    chassisVelocityTarget = speeds;
    limiter.update(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getGyro()));

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
    return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyro());
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
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // Retrieve measured ChassisSpeeds
  @Log.NT
  public ChassisSpeeds getChassisSpeeds() {
    return kSwerve.kinematics.toChassisSpeeds(getModuleStates());
  }

  // AddVisionMeasurement With Two camera streams
  public void updatePoseWithCameraData() {
    var cam1Result = camera1.getLatestResult();
    if (cam1Result.getMultiTagResult().estimatedPose.isPresent) { // checks the pose exists
      double poseAmbiguity = cam1Result.getBestTarget().getPoseAmbiguity();
      if (poseAmbiguity < 0.2
          && poseAmbiguity >= 0) { // check if the ambiguity is in the correct bounds
        Optional<EstimatedRobotPose> estimatedGlobalPoseVision = photonPoseEstimator1.update();
        poseEstimator.addVisionMeasurement(
            estimatedGlobalPoseVision.get().estimatedPose.toPose2d(),
            estimatedGlobalPoseVision.get().timestampSeconds);
      }
    }
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
  @Log.NT
  public double getGyroYawRate() {
    return Units.degreesToRadians(navX.getRawGyroZ());
  }

  public SwerveState getSwerveState() {
    return swerveState;
  }

  // ---------- Private hardware interface methods ----------

  // Get direct gyro reading as Rotation2d
  @Log.NT
  private Rotation2d getGyroRaw() {
    return navX.getRotation2d();
  }

  // Get software offset gyro angle
  @Log.NT
  private Rotation2d getGyro() {
    return getGyroRaw().minus(gyroOffset);
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
    updatePoseWithCameraData();
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

    // Multiply velocity vector by max speed
    translationVelocity = translationVelocity.times(kSwerve.maxTransSpeed);

    // Contrain velocities to boost gain
    if (!boost) translationVelocity = translationVelocity.times(kSwerve.Teleop.translationGain);
    zRotation *= kSwerve.maxAngSpeed * kSwerve.Teleop.rotationGain;

    // Construct chassis speeds and return
    return new ChassisSpeeds(
        translationVelocity.get(0, 0), translationVelocity.get(1, 0), zRotation);
  }
}
