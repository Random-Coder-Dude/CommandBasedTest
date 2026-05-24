package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Function;

/**
 * Represents a single state-transforming action in a command state machine.
 *
 * <p>An Action is a function that maps a state {@code S -> S}, optionally constrained to run only
 * when the current state matches a set of required states.
 *
 * @param <S> Enum type representing the state machine states
 */
public class Action<S extends Enum<S>> implements ActionInterface<S> {

  /** State transition function executed when this action runs */
  private final Function<S, S> function;

  /** States in which this action is allowed to run */
  private final Set<S> stateRequirements = new HashSet<>();

  /** Subsystems which the action uses */
  private final Set<SubsystemBase> subsystemRequirments = new HashSet<>();

  private final String name;

  private int priority = 0;

  /**
   * Creates an action with no state restrictions.
   *
   * @param function transformation applied when the action runs
   */
  public Action(String name, Function<S, S> function) {
    this.name = name;
    this.function = function;
  }

  /**
   * Creates an action with state restrictions.
   *
   * @param function transformation applied when the action runs
   * @param requirements allowed states for execution
   */
  @SafeVarargs
  public Action(String name, Function<S, S> function, S... requirements) {
    this.name = name;
    this.function = function;

    for (S requirement : requirements) {
      this.stateRequirements.add(requirement);
    }
  }

  /**
   * Determines whether this action is allowed to run in the given state.
   *
   * @param state current state of the command
   * @return true if runnable in this state
   */
  @Override
  public boolean canRun(S state) {
    return stateRequirements.isEmpty() || stateRequirements.contains(state);
  }

  /**
   * Executes the action logic.
   *
   * @param state current state
   * @return next state after applying transformation
   */
  @Override
  public S run(S state) {
    return function.apply(state);
  }

  /**
   * Returns the underlying function used by this action.
   *
   * @return state transition function
   */
  @Override
  public Function<S, S> getFunction() {
    return function;
  }

  @Override
  public String getName() {
    return name;
  }

  public Action<S> withPriority(int priority) {
    this.priority = Math.max(0, priority);
    return this;
  }

  public Action<S> withSubsystems(SubsystemBase... requirments) {
    for (SubsystemBase s : requirments) {
      this.subsystemRequirments.add(s);
    }
    return this;
  }

  public int getPriority() {
    return priority;
  }

  public Set<SubsystemBase> getSubsystemRequirements() {
    return subsystemRequirments;
  }
}
