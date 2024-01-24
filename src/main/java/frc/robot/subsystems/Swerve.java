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
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
  private final Field2d field2d = new Field2d();
  private final FieldObject2d autonRobot = field2d.getObject("Autonomous Pose");
  private final FieldObject2d autonPath = field2d.getObject("Autonomous Path");
  private final DoublePublisher rawGyroPub = ntTable.getDoubleTopic("Raw Gyro").publish();
  private final DoublePublisher offsetGyroPub = ntTable.getDoubleTopic("Offset Gyro").publish();
  private final DoublePublisher gyroRate = ntTable.getDoubleTopic("Gyro rate").publish();
  private final DoubleArrayPublisher chassisVelPub =
      ntTable.getDoubleArrayTopic("Commanded Chassis Velocity").publish();
  private final DoubleArrayPublisher measuredVelPub =
      ntTable.getDoubleArrayTopic("Actual Chassis Velocity").publish();
  private final DoubleArrayPublisher swerveStatesPub =
      ntTable.getDoubleArrayTopic("Swerve Module States").publish();

  // Simulation
  private final SimDeviceSim simNavX = new SimDeviceSim("navX-Sensor", 0);
  private final SimDouble simNavXYaw = simNavX.getDouble("Yaw");

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
                frontLeft.setTargetState(new SwerveModuleState(0, new Rotation2d(0)), false);
                backLeft.setTargetState(new SwerveModuleState(0, new Rotation2d(0)), false);
                frontRight.setTargetState(new SwerveModuleState(0, new Rotation2d(0)), false);
                backRight.setTargetState(new SwerveModuleState(0, new Rotation2d(0)), false);
                // apply the voltage
                frontLeft.setRawDriveVoltage(volts.magnitude());
                backLeft.setRawDriveVoltage(volts.magnitude());
                frontRight.setRawDriveVoltage(-volts.magnitude());
                backRight.setRawDriveVoltage(-volts.magnitude());
              },
              (log) -> {
                log.motor("frontLeft")
                    .voltage(m_appliedVoltage.mut_replace(frontLeft.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(frontLeft.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            frontLeft.getState().speedMetersPerSecond, MetersPerSecond));
                log.motor("backLeft")
                    .voltage(m_appliedVoltage.mut_replace(backLeft.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(backLeft.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            backLeft.getState().speedMetersPerSecond, MetersPerSecond));
                log.motor("frontRight")
                    .voltage(
                        m_appliedVoltage.mut_replace(frontRight.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(frontRight.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            frontRight.getState().speedMetersPerSecond, MetersPerSecond));
                log.motor("backRight")
                    .voltage(m_appliedVoltage.mut_replace(backRight.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(backRight.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            backRight.getState().speedMetersPerSecond, MetersPerSecond));
              },
              this));

  private final SysIdRoutine angularRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (volts) -> {
                // Set all the wheels to one direction
                frontLeft.setO();
                backLeft.setO();
                frontRight.setO();
                backRight.setO();
                // apply the voltage
                frontLeft.setRawDriveVoltage(volts.magnitude());
                backLeft.setRawDriveVoltage(volts.magnitude());
                frontRight.setRawDriveVoltage(-volts.magnitude());
                backRight.setRawDriveVoltage(-volts.magnitude());
              },
              (log) -> {
                log.motor("frontLeft")
                    .voltage(m_appliedVoltage.mut_replace(frontLeft.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(frontLeft.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            frontLeft.getState().speedMetersPerSecond, MetersPerSecond));
                log.motor("backLeft")
                    .voltage(m_appliedVoltage.mut_replace(backLeft.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(backLeft.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            backLeft.getState().speedMetersPerSecond, MetersPerSecond));
                log.motor("frontRight")
                    .voltage(
                        m_appliedVoltage.mut_replace(frontRight.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(frontRight.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            frontRight.getState().speedMetersPerSecond, MetersPerSecond));
                log.motor("backRight")
                    .voltage(m_appliedVoltage.mut_replace(backRight.getRawDriveNeoVoltage(), Volts))
                    .linearPosition(
                        m_distance.mut_replace(backRight.getPositon().distanceMeters, Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(
                            backRight.getState().speedMetersPerSecond, MetersPerSecond));
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
            () -> {
              drive(
                  fieldToRobotSpeeds(
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
                  drive(fieldToRobotSpeeds(speeds), false);
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
                  drive(fieldToRobotSpeeds(speeds), false);
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
  public SysIdRoutine getLinearRoutine(){
    return linearRoutine;
  }
  public SysIdRoutine getAngularRoutine(){
    return angularRoutine;
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
    return Units.degreesToRadians(navX.getRawGyroZ());
  }

  public SwerveState getSwerveState() {
    return swerveState;
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
    gyroRate.set(getGyroYawRate());
    // Send the chassis velocity as a double array (vel_x, vel_y, omega_z)
    chassisVelPub.set(
        new double[] {
          chassisVelocity.vxMetersPerSecond,
          chassisVelocity.vyMetersPerSecond,
          chassisVelocity.omegaRadiansPerSecond
        });
    var measuredVel = kSwerve.kinematics.toChassisSpeeds(getStates());
    measuredVelPub.set(
        new double[] {
          measuredVel.vxMetersPerSecond,
          measuredVel.vyMetersPerSecond,
          measuredVel.omegaRadiansPerSecond
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
