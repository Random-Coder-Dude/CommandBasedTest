package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Function;

/**
 * A concrete, configurable state-transforming action for use in the custom FSM command framework.
 *
 * <p>An {@code Action} wraps a {@code Function<S, S>} that maps the current FSM state to the next
 * state. Each cycle, {@link CommandRunner} evaluates all registered actions on a command, skipping
 * any whose state or subsystem requirements are not satisfied, and running the rest in
 * priority-descending order.
 *
 * <h2>State Requirements</h2>
 *
 * <p>If constructed with no state requirements, the action runs in <em>any</em> state. If one or
 * more states are provided, the action only executes when the current state matches one of them.
 *
 * <pre>{@code
 * // Runs every cycle regardless of state
 * addAction("Telemetry", state -> { log(); return state; });
 *
 * // Only runs while in INTAKING or HOLDING
 * addAction("SensorCheck", this::checkSensor, State.INTAKING, State.HOLDING);
 * }</pre>
 *
 * <h2>Priority</h2>
 *
 * <p>Actions are sorted by priority (descending) before execution each cycle. Higher-priority
 * actions run first and claim subsystems, potentially blocking lower-priority actions from running
 * due to subsystem conflicts. Default priority is {@code 0}.
 *
 * <pre>{@code
 * addAction("SafetyStop", this::eStop).withPriority(10);
 * addAction("NormalDrive", this::drive).withPriority(1);
 * }</pre>
 *
 * <h2>Subsystem Requirements</h2>
 *
 * <p>Actions can declare which {@link SubsystemBase} instances they use. If two actions in the same
 * command cycle claim the same subsystem, only the higher-priority one runs; the other is skipped
 * with reason {@code SUBSYSTEM_CONFLICT}.
 *
 * <pre>{@code
 * addAction("ArmMove", this::moveArm).withSubsystems(arm);
 * addAction("IntakeSpin", this::spinIntake).withSubsystems(intake);
 * // Both run — different subsystems, no conflict.
 * }</pre>
 *
 * @param <S> enum type representing the states of the parent command's finite state machine
 * @see ActionInterface
 * @see CommandBase
 * @see CommandRunner
 */
public class Action<S extends Enum<S>> implements ActionInterface<S> {

  /** State transition function executed when this action runs. */
  private final Function<S, S> function;

  /**
   * States in which this action is allowed to run. Empty means no restriction (runs in all states).
   */
  private final Set<S> stateRequirements = new HashSet<>();

  /** Subsystems this action claims during execution. Used for conflict detection. */
  private final Set<Subsystem> subsystemRequirements = new HashSet<>();

  /** Human-readable identifier used in telemetry log keys. */
  private final String name;

  /**
   * Execution priority. Higher values run first. Ties are broken alphabetically by action name.
   * Default is {@code 0}.
   */
  private int priority = 0;

  /**
   * Creates an action with no state restrictions — it will run in every state.
   *
   * @param name short descriptive label used in telemetry (e.g. {@code "InputHandler"})
   * @param function state transition function; receives current state, returns next state
   */
  public Action(String name, Function<S, S> function) {
    this.name = name;
    this.function = function;
  }

  /**
   * Creates an action restricted to a specific set of states.
   *
   * <p>The action will be skipped by {@link CommandRunner} in any state not listed in {@code
   * requirements}. Providing at least one requirement is recommended for actions that should only
   * apply to a particular phase of the FSM.
   *
   * @param name short descriptive label used in telemetry
   * @param function state transition function; receives current state, returns next state
   * @param requirements one or more states in which this action is eligible to run
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
   * Returns whether this action may execute in the given state.
   *
   * <p>Returns {@code true} if no state requirements were declared (unrestricted), or if {@code
   * state} is contained in the declared requirements set.
   *
   * @param state the current FSM state
   * @return {@code true} if this action is eligible; {@code false} to skip
   */
  @Override
  public boolean canRun(S state) {
    return stateRequirements.isEmpty() || stateRequirements.contains(state);
  }

  /**
   * Applies this action's state transition function and returns the resulting state.
   *
   * <p>Only called by {@link CommandRunner} after {@link #canRun(Enum)} returns {@code true} and no
   * subsystem conflict is detected. Perform hardware interaction, sensor reads, or any per-cycle
   * logic here.
   *
   * @param state the current FSM state at the time of execution
   * @return the next FSM state; return {@code state} unchanged if no transition should occur
   */
  @Override
  public S run(S state) {
    return function.apply(state);
  }

  /**
   * Returns the raw state-transition function backing this action.
   *
   * @return the {@code Function<S, S>} provided at construction time
   */
  @Override
  public Function<S, S> getFunction() {
    return function;
  }

  /**
   * Returns the telemetry name of this action.
   *
   * <p>This value is used as part of the AdvantageKit log key: {@code
   * Command/<CommandName>/Action/<name>/...}. Names should be unique within a command.
   *
   * @return this action's name
   */
  @Override
  public String getName() {
    return name;
  }

  /**
   * Sets the execution priority of this action. Higher values run earlier in each cycle.
   *
   * <p>When multiple actions are eligible in a given cycle, they are sorted by priority
   * (descending). Actions with equal priority are ordered alphabetically by name. Subsystem claims
   * are made in this sorted order, so higher-priority actions can block lower-priority ones from
   * running via {@code SUBSYSTEM_CONFLICT}.
   *
   * <p>Priority is clamped to a minimum of {@code 0}.
   *
   * @param priority non-negative integer priority; values below 0 are treated as 0
   * @return this action (fluent/builder style)
   */
  public Action<S> withPriority(int priority) {
    this.priority = Math.max(0, priority);
    return this;
  }

  /**
   * Declares the subsystems this action requires during execution.
   *
   * <p>If another action in the same command cycle has already claimed one of these subsystems
   * (because it ran first due to higher priority), this action will be skipped with reason {@code
   * SUBSYSTEM_CONFLICT}. Declaring subsystems is optional but recommended for any action that
   * directly controls hardware.
   *
   * @param requirements one or more {@link SubsystemBase} instances this action uses
   * @return this action (fluent/builder style)
   * @throws RuntimeException if any element in {@code requirements} is {@code null}
   */
  public Action<S> withSubsystems(Subsystem... requirements) {
    for (Subsystem s : requirements) {
      if (s == null)
        throw new RuntimeException("Null subsystem passed to withSubsystems in action: " + name);
      this.subsystemRequirements.add(s);
    }
    return this;
  }

  /**
   * Returns this action's execution priority.
   *
   * @return non-negative integer priority; default is {@code 0}
   */
  public int getPriority() {
    return priority;
  }

  /**
   * Returns the set of subsystems this action has declared as requirements.
   *
   * @return the set of subsystem requirements; may be empty
   */
  public Set<Subsystem> getSubsystemRequirements() {
    return Set.copyOf(subsystemRequirements);
  }
}
