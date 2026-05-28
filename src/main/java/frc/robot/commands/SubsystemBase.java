package frc.robot.commands;

/**
 * Core interface for a hardware subsystem in the custom FSM command framework.
 *
 * <p>Implement this interface for every physical mechanism on the robot (e.g. drivetrain, arm,
 * intake). Subsystem instances are passed to {@link Action#withSubsystems} to declare hardware
 * ownership, enabling {@link CommandRunner} to detect and prevent conflicting access within a
 * single cycle.
 *
 * <h2>Periodic Execution</h2>
 *
 * <p>{@link #periodic()} is called once per robot cycle by {@link CommandRunner#run()}, before
 * any command actions are evaluated. Use it for sensor reads, odometry updates, telemetry, or
 * any logic that must run unconditionally every loop regardless of FSM state.
 *
 * <h2>Conflict Detection</h2>
 *
 * <p>When two actions in the same command cycle declare the same subsystem, the lower-priority
 * action is skipped with reason {@code SUBSYSTEM_CONFLICT}. This enforces exclusive hardware
 * access without requiring any extra logic in the actions themselves.
 *
 * @see Action#withSubsystems
 * @see CommandRunner
 */
public interface SubsystemBase {

  /**
   * Returns the human-readable name of this subsystem.
   *
   * <p>Used by {@link CommandRunner} as part of the AdvantageKit telemetry key when logging
   * subsystem claims and conflicts. Should be unique across all registered subsystems.
   *
   * @return a short non-null identifier for this subsystem (e.g. {@code "DriveSubsystem"})
   */
  String getName();

  /**
   * Executes unconditional per-cycle logic for this subsystem.
   *
   * <p>Called by {@link CommandRunner#run()} once per robot cycle, before any command actions
   * are evaluated. Suitable for sensor reads, odometry updates, motor safety checks, or
   * any telemetry that should always run regardless of FSM state.
   */
  void periodic();
}
