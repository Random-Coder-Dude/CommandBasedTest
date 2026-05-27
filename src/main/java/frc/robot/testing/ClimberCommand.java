package frc.robot.testing;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CommandBase;

/**
 * Example FSM command demonstrating a sequential climber with a high-priority safety interlock.
 *
 * <p>This example showcases:
 *
 * <ul>
 *   <li><b>Priority-based safety override:</b> {@code EmergencyStop} has priority {@code 100} and
 *       runs in <em>every</em> state (no state restriction). If an unsafe condition is detected, it
 *       transitions the FSM to {@code FAULT} and claims the climber subsystem, which blocks all
 *       lower-priority actions from running that cycle via {@code SUBSYSTEM_CONFLICT}.
 *   <li><b>Sequential state gating:</b> The robot can only retract after fully extending. {@code
 *       RetractLogic} is restricted to {@code RETRACTING}, which is only reachable from {@code
 *       EXTENDED} — the FSM enforces the correct sequence.
 *   <li><b>Unrestricted cross-cutting action:</b> {@code EmergencyStop} demonstrates the
 *       no-requirements variant of {@code addAction}, which runs every cycle regardless of state.
 *       Use this pattern for safety monitors, telemetry, or any global invariant check.
 * </ul>
 *
 * <h2>State Diagram</h2>
 *
 * <pre>
 *  [IDLE] ---(start button)---> [EXTENDING] ---(at limit)---> [EXTENDED]
 *  [EXTENDED] ---(back button)---> [RETRACTING] ---(at limit)---> [CLIMBED]
 *  Any state ---(overcurrent)---> [FAULT]
 * </pre>
 *
 * <h2>Actions</h2>
 *
 * <ul>
 *   <li>{@code EmergencyStop} (priority 100, climber subsystem, all states) — detects unsafe
 *       conditions and forces FAULT, blocking everything else via subsystem conflict.
 *   <li>{@code InputHandler} (priority 0, no subsystem) — drives IDLE/EXTENDED transitions from
 *       button input.
 *   <li>{@code ExtendLogic} (priority 5, climber subsystem) — drives arm upward until limit.
 *   <li>{@code RetractLogic} (priority 5, climber subsystem) — retracts arm to climb.
 * </ul>
 *
 * @see frc.robot.commands.CommandBase
 * @see frc.robot.commands.Action#withPriority(int)
 */
public class ClimberCommand extends CommandBase<ClimberCommand.State> {

  /** FSM states for climber control. */
  public enum State {
    /** Arms stowed, waiting for climb initiation. */
    IDLE,
    /** Arms extending upward toward the bar. */
    EXTENDING,
    /** Arms fully extended, ready to retract. */
    EXTENDED,
    /** Arms retracting to pull the robot up. */
    RETRACTING,
    /** Robot fully climbed, arms at retract limit. */
    CLIMBED,
    /** Unsafe condition detected; all motion stopped. */
    FAULT
  }

  private final SubsystemBase climberSubsystem;
  private final CommandXboxController controller;

  // Simulated sensor flags — in real code these come from limit switches / current sensors
  private boolean atExtendLimit = false;
  private boolean atRetractLimit = false;
  private boolean overcurrentDetected = false;

  /**
   * Constructs the ClimberCommand and registers all actions.
   *
   * @param climberSubsystem the subsystem controlling the climber motors
   * @param controller the driver controller providing button input
   */
  public ClimberCommand(SubsystemBase climberSubsystem, CommandXboxController controller) {
    this.climberSubsystem = climberSubsystem;
    this.controller = controller;

    setCurrentState(State.IDLE);

    // Unrestricted safety monitor — runs in EVERY state before anything else.
    // Priority 100 ensures it always executes first and can claim the subsystem,
    // causing SUBSYSTEM_CONFLICT for all lower-priority actions this cycle.
    addAction("EmergencyStop", this::emergencyStop)
        .withSubsystems(climberSubsystem)
        .withPriority(100);

    // Handles IDLE -> EXTENDING and EXTENDED -> RETRACTING from button input.
    addAction("InputHandler", this::handleInput, State.IDLE, State.EXTENDED);

    // Drives the climber arm upward; transitions to EXTENDED at the limit switch.
    addAction("ExtendLogic", this::extend, State.EXTENDING)
        .withSubsystems(climberSubsystem)
        .withPriority(5);

    // Retracts the arm to pull the robot up; transitions to CLIMBED at the retract limit.
    addAction("RetractLogic", this::retract, State.RETRACTING)
        .withSubsystems(climberSubsystem)
        .withPriority(5);
  }

  /**
   * Monitors for unsafe conditions and forces {@code FAULT} if one is detected.
   *
   * <p>Runs in every state at priority 100. When this action transitions to {@code FAULT} and
   * claims the climber subsystem, all other actions that also require the climber will be skipped
   * this cycle with reason {@code SUBSYSTEM_CONFLICT}.
   *
   * @param state the current FSM state
   * @return {@code FAULT} if an unsafe condition is detected; otherwise {@code state} unchanged
   */
  private State emergencyStop(State state) {
    if (overcurrentDetected) {
      System.out.println("[Climber] FAULT: overcurrent detected — all motion stopped");
      // climberMotor.stopMotor();
      return State.FAULT;
    }
    return state;
  }

  /**
   * Reads controller input and drives transitions from {@code IDLE} and {@code EXTENDED}.
   *
   * @param state the current FSM state
   * @return the next state based on button input
   */
  private State handleInput(State state) {
    if (state == State.IDLE && controller.getHID().getStartButton()) {
      return State.EXTENDING;
    }
    if (state == State.EXTENDED && controller.getHID().getBackButton()) {
      return State.RETRACTING;
    }
    return state;
  }

  /**
   * Drives the climber arm upward and transitions to {@code EXTENDED} at the limit.
   *
   * @param state the current FSM state (always {@code EXTENDING})
   * @return {@code EXTENDED} if the extend limit is reached; otherwise {@code EXTENDING}
   */
  private State extend(State state) {
    // climberMotor.set(0.8);
    System.out.println("[Climber] EXTENDING");
    if (atExtendLimit) return State.EXTENDED;
    return state;
  }

  /**
   * Retracts the arm to pull the robot up and transitions to {@code CLIMBED} at the limit.
   *
   * @param state the current FSM state (always {@code RETRACTING})
   * @return {@code CLIMBED} if the retract limit is reached; otherwise {@code RETRACTING}
   */
  private State retract(State state) {
    // climberMotor.set(-0.8);
    System.out.println("[Climber] RETRACTING");
    if (atRetractLimit) return State.CLIMBED;
    return state;
  }
}
