package frc.robot.commands;

import java.util.function.Function;

/**
 * Contract for a state-based action in a command system.
 *
 * @param <S> Enum type representing states
 */
public interface ActionInterface<S extends Enum<S>> {

  /**
   * Determines if the action can execute in the given state.
   *
   * @param state current command state
   * @return true if action is allowed
   */
  boolean canRun(S state);

  /**
   * Executes the action and returns the next state.
   *
   * @param state current state
   * @return updated state
   */
  S run(S state);

  /**
   * Returns the underlying transformation function.
   *
   * @return state transition function
   */
  Function<S, S> getFunction();

  /**
   * Gets name of the Action
   */
  String getName();
}