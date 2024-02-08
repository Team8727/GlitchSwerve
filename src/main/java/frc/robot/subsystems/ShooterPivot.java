package frc.robot.subsystems;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kShooter.kPivot;

public class ShooterPivot extends SubsystemBase {
  // Motorcontrollers
  private CANSparkMax pivotMotor1;
  private CANSparkMax pivotMotor2;

  // Feedforwards
  private ArmFeedforward pivotFF;

  // PID controllers
  private final SparkPIDController pivotPID;

  // Absolute Encoders
  private final AbsoluteEncoder pivotEncoder;

  // Profile Stuff
  private TrapezoidProfile.Constraints constraints =
      new Constraints(kPivot.kProfile.maxVel, kPivot.kProfile.minVel);
  private TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  public ShooterPivot() {
    // Motor Initializations
    pivotMotor1 = getSparkMax(kPivot.pivot1MotorID, CANSparkLowLevel.MotorType.kBrushless);
    pivotMotor2 = getSparkMax(kPivot.pivot2MotorID, CANSparkLowLevel.MotorType.kBrushless);
    pivotMotor1.setIdleMode(CANSparkBase.IdleMode.kBrake);
    pivotMotor2.setIdleMode(CANSparkBase.IdleMode.kBrake);
    pivotMotor2.follow(pivotMotor1);

    // Feed Forwards
    pivotFF = new ArmFeedforward(kPivot.kS, kPivot.kG, kPivot.kV, kPivot.kA);

    // Encoder Configs
    pivotEncoder = pivotMotor1.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(kPivot.intakePivotEncoderPositionFactor);
    pivotEncoder.setInverted(false);

    // PID Configs
    pivotPID = pivotMotor1.getPIDController();
    pivotPID.setFeedbackDevice(pivotEncoder);
    pivotPID.setOutputRange(kPivot.minPIDOutput, kPivot.maxPIDOutput);
    pivotPID.setPositionPIDWrappingEnabled(false);
    pivotPID.setP(kPivot.kP);
    pivotPID.setD(kPivot.kD);
  }

  private TrapezoidProfile.State calculateSetpoint(double posRad) {
    goal = new TrapezoidProfile.State(posRad, 0);
    setpoint = profile.calculate(kPivot.period, setpoint, goal);
    return setpoint;
  }

  private boolean getDone() {
    return goal.position == pivotEncoder.getPosition();
  }

  // Set position input radians
  public Command setIntakePivotPos(double posRad) {
    return this.run(
            () -> {
              TrapezoidProfile.State pos = calculateSetpoint(posRad);
              pivotPID.setReference(
                  pos.position,
                  ControlType.kPosition,
                  0,
                  pivotFF.calculate(pos.position, pos.velocity));
            })
        .until(this::getDone);
  }
}
