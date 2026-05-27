package frc.robot.commands;

import java.util.function.Function;

/**
 * Core contract for a single state-transforming action within the custom FSM command framework.
 *
 * <p>Every action in the system implements this interface. An action represents one discrete
 * behavior that can inspect the current state, optionally decide to mutate it, and return the
 * resulting state. Actions are collected by a {@link CommandBase} and executed each robot cycle by
 * {@link CommandRunner}.
 *
 * <p><b>State machine contract:</b> An action must be a pure state transformer — it receives the
 * current state {@code S} and returns the next state {@code S}. Returning the same state value
 * signals no transition occurred.
 *
 * <p><b>Execution eligibility:</b> Before {@link #run(Enum)} is called, the runner checks {@link
 * #canRun(Enum)} to determine whether this action is allowed to execute in the current state.
 * Actions that fail this check are skipped for that cycle.
 *
 * @param <S> enum type representing the set of states in this command's finite state machine
 * @see Action
 * @see CommandBase
 * @see CommandRunner
 */
public interface ActionInterface<S extends Enum<S>> {

  /**
   * Determines whether this action is eligible to run in the given state.
   *
   * <p>Implementations should return {@code true} if the action has no state restrictions, or if
   * {@code state} is among the action's declared required states. The runner uses this to skip
   * actions that are not valid for the current FSM state.
   *
   * @param state the current state of the command before this action runs
   * @return {@code true} if the action may execute; {@code false} to skip it this cycle
   */
  boolean canRun(S state);

  /**
   * Executes the action's logic and returns the resulting state.
   *
   * <p>This method is only invoked by {@link CommandRunner} after {@link #canRun(Enum)} returns
   * {@code true}. Implementations should perform their behavior (e.g. motor control, sensor reads)
   * and return the state the FSM should transition to. Returning the same {@code state} value
   * indicates no state change.
   *
   * @param state the current state at the time this action executes
   * @return the next state; may be the same as {@code state} if no transition is needed
   */
  S run(S state);

  /**
   * Returns the underlying state-transition function wrapped by this action.
   *
   * <p>Primarily used for introspection, testing, or composing actions. The returned function is
   * equivalent to calling {@link #run(Enum)} directly.
   *
   * @return the {@code Function<S, S>} that implements this action's state transition logic
   */
  Function<S, S> getFunction();

  /**
   * Returns a human-readable identifier for this action.
   *
   * <p>Used by {@link CommandRunner} when writing telemetry to AdvantageKit logs under the {@code
   * Command/<CommandName>/Action/<name>/} key prefix. Should be unique within a command to avoid
   * log key collisions.
   *
   * @return a short descriptive name for this action (e.g. {@code "InputHandler"}, {@code
   *     "DriveLogic"})
   */
  String getName();
}
