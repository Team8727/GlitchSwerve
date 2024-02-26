package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimber;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase {

  CANSparkMax climbMotor;
  SimpleMotorFeedforward feedforward;
  SparkPIDController climbPID;
  LinearFilter currentFilter;

  public Climber() {
    feedforward = new SimpleMotorFeedforward(kClimber.kS, kClimber.kV, kClimber.kA);
    climbMotor = new CANSparkMax(kClimber.climberID, MotorType.kBrushless);
    // climbMotor.getOutputCurrent();
    feedforward = new SimpleMotorFeedforward(kClimber.kS, kClimber.kV, kClimber.kA);
    climbPID = climbMotor.getPIDController();
    climbPID.setP(kClimber.kP);
    climbPID.setD(kClimber.kD);

    // -=-=-=- Change timeConstant and period to Fit Robot Parameters and Desired Function: -=-=-=-

    currentFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  }

  //

  public boolean getCurrentLimitBoolean() {
    if (currentFilter.calculate(climbMotor.getOutputCurrent()) > kClimber.currentLimit) {
      return true;
    } else {
      return false;
    }
  }

  public Command driveClimbMotor(DoubleSupplier velocity) {
    return this.runOnce(() -> climbMotor.setVoltage(feedforward.calculate(velocity.getAsDouble())));
  }
}

