package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.SwerveState;
import frc.robot.utilities.LEDAnimations;
import frc.robot.utilities.LEDSubStrip;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.Supplier;

public class Indications extends SubsystemBase {
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(64);
  private final LEDSubStrip ledStrip = new LEDSubStrip(ledBuffer, 0, 63);
  private final List<Rule> rules = new ArrayList<Rule>();
  private final LEDAnimations animations = new LEDAnimations();
  private final Supplier<SwerveState> swerveStateSupplier;

  private class Rule {
    public final int priority;
    private Predicate<SystemState> ruleCondition;
    private Consumer<LEDSubStrip> action;

    public Rule(int priority, Predicate<SystemState> ruleCondition, Consumer<LEDSubStrip> action) {
      this.priority = priority;
      this.ruleCondition = ruleCondition;
      this.action = action;
    }

    public boolean evaluate(SystemState state) {
      return ruleCondition.test(state);
    }

    public void apply(LEDSubStrip strip) {
      action.accept(strip);
    }
  }

  private class SystemState {
    public SwerveState swerveState;

    public SystemState(SwerveState swerveState) {
      this.swerveState = swerveState;
    }
  }

  public Indications(Supplier<SwerveState> swerveStateSupplier) {
    this.swerveStateSupplier = swerveStateSupplier;
    rules.add(
        new Rule(
            100,
            state -> state.swerveState.boosting,
            strip -> animations.monotone(strip, Color.kAliceBlue)));
    rules.add(
        new Rule(
            0,
            state -> state.swerveState.mode == SwerveState.Mode.IDLE,
            strip -> animations.monotone(strip, Color.kBlack)));
    rules.add(
        new Rule(
            20,
            state -> state.swerveState.mode == SwerveState.Mode.PARK,
            strip -> animations.monotone(strip, Color.kRed)));

    sortRules(rules);
  }

  @Override
  public void periodic() {
    // Update data
    SystemState state = new SystemState(swerveStateSupplier.get());
    animations.update();

    // Apply rules
    evaluateRules(rules, ledStrip, state);
  }

  // Evaluate the rules in rank order and apply the first valid rule to the strip
  private void evaluateRules(List<Rule> rules, LEDSubStrip strip, SystemState state) {
    for (var rule : rules) {
      if (rule.evaluate(state)) {
        rule.apply(strip);
      }
    }
  }

  // Sort the rules in descending rank
  private void sortRules(List<Rule> rules) {
    Collections.sort(rules, (rule1, rule2) -> rule2.priority - rule1.priority);
  }
}
