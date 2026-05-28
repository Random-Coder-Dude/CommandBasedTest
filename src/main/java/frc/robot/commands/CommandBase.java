package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * Abstract base class for all FSM-based commands in the custom command framework.
 *
 * <p>{@code CommandBase} provides the standard implementation of {@link CommandInterface} and is
 * the intended starting point for every command you write. Extend it, declare your state enum,
 * call {@link #setCurrentState} with an initial state, and register your actions via
 * {@link #addAction} — all within the constructor.
 *
 * <h2>Typical Usage Pattern</h2>
 * <pre>{@code
 * public class ShooterCommand extends CommandBase<ShooterCommand.State> {
 *
 *     public enum State { IDLE, SPINNING_UP, READY, FIRING }
 *
 *     public ShooterCommand(CommandXboxController controller) {
 *         setCurrentState(State.IDLE);
 *
 *         addAction("InputHandler", this::handleInput,
 *             State.IDLE, State.SPINNING_UP, State.READY);
 *         addAction("SpinUpLogic", this::spinUp, State.SPINNING_UP);
 *         addAction("FireLogic",   this::fire,   State.FIRING);
 *     }
 * }
 * }</pre>
 *
 * <h2>Action Execution Model</h2>
 * <p>Each robot cycle, {@link CommandRunner} processes the command by:
 * <ol>
 *   <li>Sorting the action list by priority (descending), then alphabetically by name.</li>
 *   <li>Iterating actions and checking {@link Action#canRun(Enum)} against the current state.</li>
 *   <li>Checking for subsystem conflicts with previously-claimed subsystems this cycle.</li>
 *   <li>Running eligible actions in order, threading the state value through each one.</li>
 *   <li>Committing the final state back via {@link #setCurrentState(Enum)}.</li>
 * </ol>
 *
 * <h2>Initial State Requirement</h2>
 * <p>{@link CommandRegisterer#register} throws a {@link RuntimeException} if
 * {@link #getCurrentState()} is {@code null} at registration time. Always call
 * {@code setCurrentState(...)} before the constructor returns.
 *
 * @param <S> enum type representing this command's finite state machine states
 *
 * @see Action
 * @see CommandRunner
 * @see CommandRegisterer
 */
public abstract class CommandBase<S extends Enum<S>> implements CommandInterface<S> {
  /** The FSM's current state, threaded through all actions each cycle. */
  private S state;

  /**
   * Ordered list of actions evaluated each robot cycle.
   * Populated via {@link #addAction} during construction.
   */
  private final List<Action<S>> actions = new ArrayList<>();

  /**
   * {@inheritDoc}
   *
   * <p>Called by {@link CommandRunner} at the end of each cycle to commit the final state.
   * Also used in the constructor to set the required initial state before registration.
   */
  @Override
  public void setCurrentState(S newState) {
    this.state = newState;
  }

  /**
   * {@inheritDoc}
   *
   * <p>Returns the state committed at the end of the last cycle, or the initial state set
   * during construction if no cycle has run yet.
   */
  @Override
  public S getCurrentState() {
    return state;
  }

  /**
   * {@inheritDoc}
   *
   * <p>Returns the live action list. {@link CommandRunner} sorts and iterates this list each
   * cycle. Do not modify this list outside the constructor.
   */
  @Override
  public List<Action<S>> getActions() {
    return List.copyOf(actions);
  }

  /**
   * {@inheritDoc}
   *
   * <p>Returns the simple class name by default (e.g. {@code "IntakeCommand"}). Override if
   * you need a custom telemetry key or if multiple commands share a class name.
   */
  @Override
  public String getName() {
    return getClass().getSimpleName();
  }

  /**
   * Registers an unrestricted action that runs every cycle regardless of state.
   *
   * <p>Use this for cross-cutting concerns like telemetry logging, safety checks, or any
   * behavior that should always be evaluated. The returned {@link Action} can be further
   * configured with {@link Action#withPriority(int)} or {@link Action#withSubsystems}.
   *
   * <pre>{@code
   * addAction("SafetyMonitor", state -> {
   *     if (atHardLimit()) return State.FAULT;
   *     return state;
   * });
   * }</pre>
   *
   * @param name   telemetry name for this action; should be unique within this command
   * @param action state transition function; receives current state, returns next state
   * @return the created {@link Action}, for optional fluent configuration
   */
  public Action<S> addAction(String name, Function<S, S> action) {
    Action<S> createdAction = new Action<>(name, action);
    actions.add(createdAction);
    return createdAction;
  }

  /**
   * Registers a state-restricted action that only runs when the current state matches one of
   * the provided requirements.
   *
   * <p>This is the most common form of action registration. Restricting actions to relevant
   * states keeps each method focused and prevents unintended side effects in other phases of
   * the FSM. The returned {@link Action} can be further configured with
   * {@link Action#withPriority(int)} or {@link Action#withSubsystems}.
   *
   * <pre>{@code
   * addAction("DriveLogic", this::drive, State.TELEOP, State.AUTO)
   *     .withPriority(5)
   *     .withSubsystems(driveSubsystem);
   * }</pre>
   *
   * @param name               telemetry name; should be unique within this command
   * @param action             state transition function
   * @param stateRequirements  one or more states in which this action may run
   * @return the created {@link Action}, for optional fluent configuration
   */
  @SafeVarargs
  public final Action<S> addAction(String name, Function<S, S> action, S... stateRequirements) {
    Action<S> createdAction = new Action<>(name, action, stateRequirements);
    actions.add(createdAction);
    return createdAction;
  }
}
