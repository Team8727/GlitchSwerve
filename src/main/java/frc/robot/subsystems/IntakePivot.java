package frc.robot.subsystems;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIntake.kPivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class IntakePivot extends SubsystemBase {

    private CANSparkMax pivotMotor;

    private SparkPIDController pivotPID;

    private AbsoluteEncoder pivotEncoder;

    private ArmFeedforward pivotFF;
    
    //Profile Stuff
    private TrapezoidProfile.Constraints m_constraints = new Constraints(kPivot.kProfile.maxVel, kPivot.kProfile.minVel);
    private TrapezoidProfile m_profile = new TrapezoidProfile(m_constraints);
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    public IntakePivot(){
        pivotMotor = getSparkMax(kPivot.pivotMotorID,CANSparkLowLevel.MotorType.kBrushless);

        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pivotEncoder.setPositionConversionFactor(kPivot.intakePivotEncoderPositionFactor);
        pivotEncoder.setInverted(false);

        pivotPID = pivotMotor.getPIDController();
        pivotPID.setOutputRange(kPivot.minPIDOutput, kPivot.maxPIDOutput);
        pivotPID.setPositionPIDWrappingEnabled(false);
        pivotPID.setP(kPivot.kP);
        pivotPID.setD(kPivot.kD);

        pivotFF = new ArmFeedforward(kPivot.kS, kPivot.kG, kPivot.kV, kPivot.kA);
    }
    //Set position input radians
    public Command setIntakePivotPos(double posRad){
        return this.run(() -> pivotPID.setReference(pivotFF.calculate(posRad, 0,0), ControlType.kPosition));
    }

}
