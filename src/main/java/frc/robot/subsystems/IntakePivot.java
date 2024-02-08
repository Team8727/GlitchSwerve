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
import frc.robot.Constants.kIntake.kPivot;

public class IntakePivot extends SubsystemBase {

  private CANSparkMax pivotMotor;

  private SparkPIDController pivotPID;

  private AbsoluteEncoder pivotEncoder;

  private ArmFeedforward pivotFF;

  // Profile Stuff
  private TrapezoidProfile.Constraints constraints =
      new Constraints(kPivot.kProfile.maxVel, kPivot.kProfile.minVel);
  private TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  public IntakePivot() {
    pivotMotor = getSparkMax(kPivot.pivotMotorID, CANSparkLowLevel.MotorType.kBrushless);
    pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(kPivot.intakePivotEncoderPositionFactor);
    pivotEncoder.setInverted(false);

    pivotPID = pivotMotor.getPIDController();
    pivotPID.setFeedbackDevice(pivotEncoder);
    pivotPID.setOutputRange(kPivot.minPIDOutput, kPivot.maxPIDOutput);
    pivotPID.setP(kPivot.kP);
    pivotPID.setD(kPivot.kD);

    pivotFF = new ArmFeedforward(kPivot.kS, kPivot.kG, kPivot.kV, kPivot.kA);
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
