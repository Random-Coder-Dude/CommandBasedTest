package frc.robot.testing;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CommandBase;

/**
 * Example FSM command demonstrating a swerve drivetrain with multiple driving modes and
 * auto-alignment that exclusively locks the drive subsystem when active.
 *
 * <p>This example showcases:
 *
 * <ul>
 *   <li><b>Multiple state-gated behaviors sharing one subsystem:</b> {@code FieldRelativeDrive},
 *       {@code RobotRelativeDrive}, and {@code SlowModeDrive} all declare the drive subsystem, but
 *       are each restricted to a different state. Because only one state is active at a time,
 *       exactly one of the three will be eligible per cycle — no conflict between them.
 *   <li><b>Auto-align blocking manual drive via subsystem conflict:</b> {@code AutoAlign} has
 *       priority {@code 10} and also declares the drive subsystem. When the FSM is in {@code
 *       AUTO_ALIGN}, {@code AutoAlign} runs first and claims the subsystem. The manual drive action
 *       for that state would also be eligible, but gets skipped with {@code SUBSYSTEM_CONFLICT} —
 *       exclusive hardware control without any extra logic.
 *   <li><b>Mode toggling via an unrestricted input handler:</b> {@code ModeSelector} runs in all
 *       drive states and handles all mode-switching logic in one place, keeping the drive action
 *       methods clean.
 * </ul>
 *
 * <h2>State Diagram</h2>
 *
 * <pre>
 *  [FIELD_RELATIVE] <---(LB toggle)---> [ROBOT_RELATIVE]
 *  [FIELD_RELATIVE] / [ROBOT_RELATIVE] ---(RB held)---> [SLOW_MODE]
 *  Any drive state ---(X held)---> [AUTO_ALIGN] ---(aligned or X released)---> [FIELD_RELATIVE]
 * </pre>
 *
 * <h2>Actions</h2>
 *
 * <ul>
 *   <li>{@code ModeSelector} (priority 0, no subsystem, all states) — toggles between drive modes
 *       and initiates auto-align.
 *   <li>{@code AutoAlign} (priority 10, drive subsystem) — PID-drives to target heading; blocks all
 *       manual drive actions via subsystem conflict.
 *   <li>{@code FieldRelativeDrive} (priority 0, drive subsystem) — standard field-relative swerve
 *       drive.
 *   <li>{@code RobotRelativeDrive} (priority 0, drive subsystem) — robot-relative swerve drive.
 *   <li>{@code SlowModeDrive} (priority 0, drive subsystem) — field-relative drive with inputs
 *       scaled to 30%.
 * </ul>
 *
 * @see frc.robot.commands.CommandBase
 * @see frc.robot.commands.Action#withSubsystems
 */
public class DriveCommand extends CommandBase<DriveCommand.State> {

  /** FSM states for drivetrain control. */
  public enum State {
    /** Standard field-relative swerve drive. */
    FIELD_RELATIVE,
    /** Robot-relative swerve drive (useful for fine positioning). */
    ROBOT_RELATIVE,
    /** Field-relative drive with inputs scaled to 30% for precise maneuvers. */
    SLOW_MODE,
    /** PID-based automatic heading alignment to a target angle. */
    AUTO_ALIGN
  }

  private final SubsystemBase driveSubsystem;
  private final CommandXboxController controller;

  /** Simulated alignment flag — in real code this comes from a PID controller's atSetpoint(). */
  private boolean isAligned = false;

  /**
   * Constructs the DriveCommand and registers all actions.
   *
   * @param driveSubsystem the subsystem controlling the swerve drivetrain
   * @param controller the driver controller providing joystick and button input
   */
  public DriveCommand(SubsystemBase driveSubsystem, CommandXboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.controller = controller;

    setCurrentState(State.FIELD_RELATIVE);

    // Handles all mode transitions. No subsystem claim — pure input logic.
    // Runs in all states so mode switches are always evaluated.
    addAction(
        "ModeSelector",
        this::selectMode,
        State.FIELD_RELATIVE,
        State.ROBOT_RELATIVE,
        State.SLOW_MODE,
        State.AUTO_ALIGN);

    // AutoAlign: higher priority than all drive actions. Claims the drive subsystem,
    // so any manual drive action that is also eligible this cycle will be skipped
    // with SUBSYSTEM_CONFLICT — effectively giving AutoAlign exclusive hardware control.
    addAction("AutoAlign", this::autoAlign, State.AUTO_ALIGN)
        .withSubsystems(driveSubsystem)
        .withPriority(10);

    // The three manual drive modes all have default priority (0) and declare the drive
    // subsystem. Only one is ever state-eligible per cycle, so they never conflict with
    // each other. All three are blocked by AutoAlign when it runs.
    addAction("FieldRelativeDrive", this::driveFieldRelative, State.FIELD_RELATIVE)
        .withSubsystems(driveSubsystem);

    addAction("RobotRelativeDrive", this::driveRobotRelative, State.ROBOT_RELATIVE)
        .withSubsystems(driveSubsystem);

    addAction("SlowModeDrive", this::driveSlowMode, State.SLOW_MODE).withSubsystems(driveSubsystem);
  }

  /**
   * Reads controller input and manages transitions between all drive modes.
   *
   * <p>X button enters {@code AUTO_ALIGN}; releasing X returns to {@code FIELD_RELATIVE}. Right
   * bumper held enters {@code SLOW_MODE}. Left bumper pressed toggles between {@code
   * FIELD_RELATIVE} and {@code ROBOT_RELATIVE}.
   *
   * @param state the current FSM state
   * @return the next drive mode state
   */
  private State selectMode(State state) {
    if (controller.getHID().getXButton()) {
      return State.AUTO_ALIGN;
    }
    if (state == State.AUTO_ALIGN) {
      return State.FIELD_RELATIVE;
    }
    if (controller.getHID().getRightBumper()) {
      return State.SLOW_MODE;
    }
    if (controller.getHID().getLeftBumperPressed()) {
      return state == State.FIELD_RELATIVE ? State.ROBOT_RELATIVE : State.FIELD_RELATIVE;
    }
    return state;
  }

  /**
   * Runs PID-based auto-alignment to a target heading.
   *
   * <p>This action has priority 10, so it runs before the manual drive actions and claims the drive
   * subsystem. Any manual drive action that is also eligible this cycle is automatically skipped
   * with {@code SUBSYSTEM_CONFLICT}.
   *
   * @param state the current FSM state (always {@code AUTO_ALIGN})
   * @return {@code FIELD_RELATIVE} when aligned; otherwise {@code AUTO_ALIGN}
   */
  private State autoAlign(State state) {
    // chassis speeds = pidController.calculate(gyro.getAngle(), targetAngle);
    // swerve.drive(chassisSpeeds);
    System.out.println("[Drive] AUTO-ALIGNING");
    if (isAligned) return State.FIELD_RELATIVE;
    return state;
  }

  /**
   * Drives the swerve in field-relative mode using full joystick input.
   *
   * @param state the current FSM state (always {@code FIELD_RELATIVE})
   * @return {@code FIELD_RELATIVE} unchanged
   */
  private State driveFieldRelative(State state) {
    double x = controller.getLeftX();
    double y = controller.getLeftY();
    double rot = controller.getRightX();
    // swerve.driveFieldRelative(x, y, rot);
    System.out.printf("[Drive] FIELD-RELATIVE (%.2f, %.2f, rot=%.2f)%n", x, y, rot);
    return state;
  }

  /**
   * Drives the swerve in robot-relative mode using full joystick input.
   *
   * @param state the current FSM state (always {@code ROBOT_RELATIVE})
   * @return {@code ROBOT_RELATIVE} unchanged
   */
  private State driveRobotRelative(State state) {
    double x = controller.getLeftX();
    double y = controller.getLeftY();
    double rot = controller.getRightX();
    // swerve.driveRobotRelative(x, y, rot);
    System.out.printf("[Drive] ROBOT-RELATIVE (%.2f, %.2f, rot=%.2f)%n", x, y, rot);
    return state;
  }

  /**
   * Drives the swerve in field-relative mode with inputs scaled to 30% for precision.
   *
   * @param state the current FSM state (always {@code SLOW_MODE})
   * @return {@code SLOW_MODE} unchanged
   */
  private State driveSlowMode(State state) {
    double x = controller.getLeftX() * 0.3;
    double y = controller.getLeftY() * 0.3;
    double rot = controller.getRightX() * 0.3;
    // swerve.driveFieldRelative(x, y, rot);
    System.out.printf("[Drive] SLOW MODE (%.2f, %.2f, rot=%.2f)%n", x, y, rot);
    return state;
  }
}
