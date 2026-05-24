package frc.robot.commands;

import java.util.List;

/**
 * Core interface for a state-driven command system.
 *
 * @param <S> enum type representing state machine states
 */
public interface CommandInterface<S extends Enum<S>> {

  /**
   * Gets the current state of the command.
   *
   * @return current state
   */
  S getCurrentState();

  /**
   * Sets the current state of the command.
   *
   * @param state new state
   */
  void setCurrentState(S state);

  /**
   * Gets all actions associated with this command.
   *
   * @return list of actions
   */
  List<Action<S>> getActions();

  /**
   * Gets the human-readable name of this command.
   *
   * @return class name or identifier
   */
  String getName();

  /**
   * Sets priority of the Runner
   *
   * @param priority the priority value (bigger value is more important)
   */
  void setPriority(int priority);

  /** Gets priority of the Runner */
  int getPriority();
}
