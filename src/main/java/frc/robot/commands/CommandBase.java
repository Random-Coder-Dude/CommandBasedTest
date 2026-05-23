package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * Base implementation of a state-driven command composed of Actions.
 *
 * <p>A CommandBase maintains a current state and executes all registered
 * actions each cycle, allowing them to mutate the state.
 *
 * @param <S> enum type representing command states
 */
public abstract class CommandBase<S extends Enum<S>> implements CommandInterface<S> {

  /** Current state of the command */
  private S state;

  /** Ordered list of actions executed each cycle */
  private final List<Action<S>> actions = new ArrayList<>();

  @Override
  public void setCurrentState(S newState) {
    this.state = newState;
  }

  @Override
  public S getCurrentState() {
    return state;
  }

  @Override
  public List<Action<S>> getActions() {
    return actions;
  }

  @Override
  public String getName() {
    return getClass().getSimpleName();
  }

  /**
   * Adds an action that always runs regardless of state.
   *
   * @param action state transformation function
   */
  public void addAction(Function<S, S> action) {
    actions.add(new Action<>(action));
  }

  /**
   * Adds an action that only runs in specific states.
   *
   * @param action state transformation function
   * @param requirements allowed states
   */
  @SafeVarargs
  public final void addAction(Function<S, S> action, S... requirements) {
    actions.add(new Action<>(action, requirements));
  }
}