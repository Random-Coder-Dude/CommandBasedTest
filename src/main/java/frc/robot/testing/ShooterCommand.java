package frc.robot.testing;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CommandBase;
import frc.robot.commands.SubsystemBase;

/**
 * Example FSM command demonstrating a two-stage shooter: spin up the flywheel first, then fire.
 *
 * <p>This example showcases:
 *
 * <ul>
 *   <li><b>State gating:</b> The robot cannot fire until the flywheel has reached target speed.
 *       {@code FireLogic} is restricted to {@code FIRING} only, so pressing the trigger while still
 *       in {@code SPINNING_UP} does nothing until {@code SpinUpLogic} transitions to {@code READY}.
 *   <li><b>Sensor-driven transitions:</b> {@code SpinUpLogic} reads a simulated RPM value and
 *       transitions to {@code READY} automatically once at speed — no button press needed.
 *   <li><b>Subsystem conflict prevention:</b> Both {@code SpinUpLogic} and {@code FireLogic}
 *       declare {@code shooterSubsystem}. Because they are state-gated to different states, they
 *       never both run in the same cycle, so no conflict occurs in practice. The declarations are
 *       still good practice for clarity and safety.
 * </ul>
 *
 * <h2>State Diagram</h2>
 *
 * <pre>
 *  [IDLE] ---(Y button)---> [SPINNING_UP] ---(at speed)---> [READY] ---(trigger > 0.5)---> [FIRING]
 *    ^                                                                                          |
 *    |_________________________(trigger released)_______________________________________________|
 * </pre>
 *
 * <h2>Actions</h2>
 *
 * <ul>
 *   <li>{@code InputHandler} (priority 0, no subsystem) — reads buttons and manages
 *       IDLE/READY/FIRING transitions.
 *   <li>{@code SpinUpLogic} (priority 5, shooter subsystem) — ramps flywheel and auto-transitions
 *       to READY once target RPM is reached.
 *   <li>{@code FireLogic} (priority 5, shooter subsystem) — drives the feed mechanism while in
 *       FIRING state.
 * </ul>
 *
 * @see frc.robot.commands.CommandBase
 * @see frc.robot.commands.CommandRunner
 */
public class ShooterCommand extends CommandBase<ShooterCommand.State> {

  /** FSM states for shooter control. */
  public enum State {
    /** Flywheel stopped, waiting for spin-up command. */
    IDLE,
    /** Flywheel ramping up toward target RPM. */
    SPINNING_UP,
    /** Flywheel at target RPM, ready to fire. */
    READY,
    /** Feed mechanism active, shooting game pieces. */
    FIRING
  }

  private final SubsystemBase shooterSubsystem;
  private final CommandXboxController controller;

  /** Simulated flywheel speed. In real code this would come from a motor encoder. */
  private double currentRPM = 0;

  private static final double TARGET_RPM = 4000;

  /**
   * Constructs the ShooterCommand and registers all actions.
   *
   * @param shooterSubsystem the subsystem controlling the flywheel and feed motors
   * @param controller the driver controller providing button input
   */
  public ShooterCommand(SubsystemBase shooterSubsystem, CommandXboxController controller) {
    this.shooterSubsystem = shooterSubsystem;
    this.controller = controller;

    setCurrentState(State.IDLE);

    // Handles IDLE -> SPINNING_UP, READY -> FIRING, and FIRING -> IDLE transitions.
    // No subsystem claim — this action only reads buttons and returns a new state.
    addAction("InputHandler", this::handleInput, State.IDLE, State.READY, State.FIRING);

    // Ramps the flywheel and transitions to READY once at speed.
    addAction("SpinUpLogic", this::spinUp, State.SPINNING_UP)
        .withSubsystems(shooterSubsystem)
        .withPriority(5);

    // Runs the feed mechanism to push game pieces into the spinning flywheel.
    addAction("FireLogic", this::fire, State.FIRING)
        .withSubsystems(shooterSubsystem)
        .withPriority(5);
  }

  /**
   * Reads controller input and drives state transitions for IDLE, READY, and FIRING.
   *
   * @param state the current FSM state
   * @return the next state based on button input
   */
  private State handleInput(State state) {
    if (state == State.IDLE && controller.getHID().getYButton()) {
      return State.SPINNING_UP;
    }
    if (state == State.READY && controller.getRightTriggerAxis() > 0.5) {
      return State.FIRING;
    }
    if (state == State.FIRING && controller.getRightTriggerAxis() <= 0.5) {
      return State.IDLE;
    }
    return state;
  }

  /**
   * Ramps the flywheel toward target RPM and transitions to {@code READY} once at speed.
   *
   * @param state the current FSM state (always {@code SPINNING_UP})
   * @return {@code READY} if target RPM reached; otherwise {@code SPINNING_UP}
   */
  private State spinUp(State state) {
    currentRPM += 200; // In real code: shooterMotor.set(1.0); currentRPM = encoder.getRate();
    if (currentRPM >= TARGET_RPM) {
      currentRPM = TARGET_RPM;
      return State.READY;
    }
    return state;
  }

  /**
   * Drives the feed mechanism to push game pieces into the spinning flywheel.
   *
   * @param state the current FSM state (always {@code FIRING})
   * @return {@code FIRING} unchanged — transition back to IDLE is handled by InputHandler
   */
  private State fire(State state) {
    // feedMotor.set(1.0);
    System.out.println("[Shooter] FIRING at " + currentRPM + " RPM");
    return state;
  }
}
