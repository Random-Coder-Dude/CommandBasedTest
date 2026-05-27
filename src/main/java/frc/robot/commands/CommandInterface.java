package frc.robot.commands;

import java.util.List;

/**
 * Core interface for a state-driven command in the custom FSM framework.
 *
 * <p>A {@code CommandInterface} represents a single logical robot behavior (e.g. controlling an
 * intake, an arm, or a drivetrain) modeled as a finite state machine. It owns a current state and a
 * list of {@link Action} objects that are evaluated each robot cycle by {@link CommandRunner}.
 *
 * <p>Implementations should use {@link CommandBase} rather than implementing this interface
 * directly. Direct implementation is intended for advanced use cases such as testing or alternative
 * base classes.
 *
 * <h2>Lifecycle</h2>
 *
 * <ol>
 *   <li>A concrete command is instantiated and registers its actions in its constructor via {@link
 *       CommandBase#addAction}.
 *   <li>The command is registered with {@link CommandRegisterer#register} (typically inside {@code
 *       RobotContainer}).
 *   <li>Each robot cycle, {@link CommandRunner#run()} iterates all registered commands and calls
 *       {@link #getActions()} and {@link #getCurrentState()} to evaluate the action pipeline.
 *   <li>{@link #setCurrentState(Enum)} is called by the runner after all eligible actions have
 *       executed, committing the final state for that cycle.
 * </ol>
 *
 * @param <S> enum type representing the set of states in this command's finite state machine
 * @see CommandBase
 * @see CommandRunner
 * @see CommandRegisterer
 */
public interface CommandInterface<S extends Enum<S>> {

  /**
   * Returns the command's current FSM state.
   *
   * <p>This value is read by {@link CommandRunner} at the start of each cycle to determine which
   * actions are eligible to run. It reflects the state committed at the end of the previous cycle
   * (or the initial state if no cycle has run yet).
   *
   * @return the current state; must not be {@code null} after construction
   */
  S getCurrentState();

  /**
   * Updates the command's current FSM state.
   *
   * <p>Called by {@link CommandRunner} at the end of each cycle to commit the final state produced
   * by the action pipeline. This should generally not be called from within actions themselves —
   * state transitions are expressed by returning a new state from {@link Action#run(Enum)}.
   *
   * @param state the new state to commit; must not be {@code null}
   */
  void setCurrentState(S state);

  /**
   * Returns the list of all actions registered to this command.
   *
   * <p>{@link CommandRunner} reads this list each cycle, sorts it by priority, checks each action's
   * state and subsystem eligibility, and runs the ones that pass. The list is mutable during
   * construction (via {@link CommandBase#addAction}) but should be treated as stable after the
   * command is registered.
   *
   * @return the live, ordered list of actions; never {@code null}
   */
  List<Action<S>> getActions();

  /**
   * Returns a human-readable name for this command.
   *
   * <p>Used by {@link CommandRunner} as the top-level AdvantageKit log key prefix: {@code
   * Command/<name>/...}. By default, {@link CommandBase} returns the simple class name. Override to
   * use a custom identifier, especially if two command classes share a name.
   *
   * @return a short non-null identifier for this command (e.g. {@code "IntakeCommand"})
   */
  String getName();
}
