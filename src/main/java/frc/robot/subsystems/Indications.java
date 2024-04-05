package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kIndications;
import frc.robot.Constants.kIntake.kPivot.IntakePosition;
import frc.robot.utilities.LEDAnimations;
import frc.robot.utilities.LEDSubStrip;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.BooleanSupplier;

public class Indications extends SubsystemBase {
  // Leds
  private final AddressableLED leds = new AddressableLED(kIndications.ledPort);
  private final AddressableLEDBuffer ledBuffer =
      new AddressableLEDBuffer(kIndications.leftStripLength + kIndications.rightStripLength);
  private final LEDSubStrip leftStrip =
      new LEDSubStrip(ledBuffer, 0, kIndications.leftStripLength - 1);
  private final LEDSubStrip rightStrip =
      new LEDSubStrip(ledBuffer, kIndications.leftStripLength, ledBuffer.getLength() - 1);

  // Dependcies
  @SuppressWarnings("unused")
  private final Swerve swerve;

  private final IntakePivot intakePivot;

  @SuppressWarnings("unused")
  private final ShooterPivot shooterPivot;

  private final IntakeRollers intakeRollers;
  private final HandoffRollers handoffRollers;

  @SuppressWarnings("unused")
  private final ShooterFlywheels shooterFlywheels;

  private final CommandXboxController driverController;

  // Indications Objects
  private final List<Rule> ledsRules = new ArrayList<Rule>();
  private final LEDAnimations animations = new LEDAnimations();

  private class Rule {
    public final int priority;
    private BooleanSupplier ruleCondition;
    private Runnable action;

    public Rule(int priority, BooleanSupplier ruleCondition, Runnable action) {
      this.priority = priority;
      this.ruleCondition = ruleCondition;
      this.action = action;
    }

    public boolean evaluate() {
      return ruleCondition.getAsBoolean();
    }

    public void apply() {
      action.run();
    }
  }

  public Indications(
      Swerve swerve,
      IntakePivot intakePivot,
      ShooterPivot shooterPivot,
      IntakeRollers intakeRollers,
      HandoffRollers handoffRollers,
      ShooterFlywheels shooterFlywheels,
      CommandXboxController driverController) {
    this.swerve = swerve;
    this.intakePivot = intakePivot;
    this.shooterPivot = shooterPivot;
    this.intakeRollers = intakeRollers;
    this.handoffRollers = handoffRollers;
    this.shooterFlywheels = shooterFlywheels;
    this.driverController = driverController;

    populateRules();
    bindTriggers();
    sortRules(ledsRules);

    leds.setLength(ledBuffer.getLength());
    leds.setData(ledBuffer);
    leds.start();
  }

  private void populateRules() {
    ledsRules.add(
        new Rule(
            120,
            () ->
                (swerve.getCurrentCommand() != null ? swerve.getCurrentCommand().getName() : "")
                    == "autoAmp",
            () -> {
              animations.rainbow(leftStrip, 1);
              animations.rainbow(rightStrip, 1);
            }));

    ledsRules.add(
        new Rule(
            100,
            () -> handoffRollers.hasPiece(),
            () -> {
              animations.monotone(leftStrip, Color.kGreen);
              animations.monotone(rightStrip, Color.kGreen);
            }));

    ledsRules.add(
        new Rule(
            80,
            () -> intakeRollers.hasPiece(),
            () -> {
              animations.monotone(leftStrip, Color.kOrange);
              animations.monotone(rightStrip, Color.kOrange);
            }));

    ledsRules.add(
        new Rule(
            60,
            () -> intakePivot.getGoalPosition() == IntakePosition.DEPLOYED,
            () -> {
              animations.flashing(leftStrip, Color.kBlue, 12);
              animations.flashing(rightStrip, Color.kBlue, 12);
            }));

    ledsRules.add(
        new Rule(
            0,
            () -> true,
            () -> {
              animations.alternate(leftStrip, Color.kBlack, Color.kPink);
              animations.alternate(rightStrip, Color.kBlack, Color.kPink);
            }));
  }

  private void bindTriggers() {
    new Trigger(intakeRollers::isIndexing)
        .onTrue(
            Commands.startEnd(
                    () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1),
                    () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0))
                .withTimeout(0.3));
  }

  @Override
  public void periodic() {
    // Update data
    animations.update();

    // Apply rules
    evaluateRules(ledsRules);

    // Update leds
    leds.setData(ledBuffer);
  }

  // Evaluate the rules in rank order and apply the first valid rule to the strip
  private void evaluateRules(List<Rule> rules) {
    for (var rule : rules) {
      if (rule.evaluate()) {
        rule.apply();
        break;
      }
    }
  }

  // Sort the rules in descending rank
  private void sortRules(List<Rule> rules) {
    Collections.sort(rules, (rule1, rule2) -> rule2.priority - rule1.priority);
  }
}
