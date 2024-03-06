package frc.robot.subsystems;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimber;
import frc.robot.utilities.SparkConfigurator.LogData;
import java.util.Set;

public class Climber extends SubsystemBase {

  private final CANSparkMax climbMotor;
  private final RelativeEncoder climbEncoder;
  private final SimpleMotorFeedforward feedforward;
  private final SparkPIDController climbPID;
  private LinearFilter currentFilter;

  public Climber() {
    climbMotor =
        getSparkMax(
            kClimber.climberID, MotorType.kBrushless, false, Set.of(), Set.of(LogData.VOLTAGE));
    climbMotor.setIdleMode(IdleMode.kBrake);
    climbEncoder = climbMotor.getEncoder();
    feedforward = new SimpleMotorFeedforward(kClimber.kS, kClimber.kV, kClimber.kA);
    climbPID = climbMotor.getPIDController();
    climbPID.setP(kClimber.kP);
    climbPID.setD(kClimber.kD);

    // -=-=-=- Change timeConstant and period to Fit Robot Parameters and Desired Function: -=-=-=-

    currentFilter = LinearFilter.singlePoleIIR(kClimber.timeConstant, kClimber.period);
  }

  //

  public boolean getIfCurrentLimit() {
    return (currentFilter.calculate(climbMotor.getOutputCurrent()) > kClimber.currentLimit);
  }

  public Command climbUp(double velocity) {
    return run(() -> climbMotor.setVoltage(velocity))
        .until(() -> climbEncoder.getPosition() >= kClimber.rotationsToClimb)
        .finallyDo(() -> climbMotor.setVoltage(0));
  }

  public Command climbDown(double velocity) {
    return run(() -> climbMotor.setVoltage(-velocity))
        .until(() -> climbEncoder.getPosition() <= 0)
        .finallyDo(() -> climbMotor.setVoltage(0));
  }
}
