package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

/**
 * Global registry that tracks every active command in the robot program.
 *
 * <p>{@code CommandRegisterer} is a static registry — there is one shared list of commands for the
 * entire robot. Commands are added via {@link #register} (typically in {@code RobotContainer}) and
 * then retrieved each cycle by {@link CommandRunner#run()} for execution.
 *
 * <h2>Registration Rules</h2>
 *
 * <ul>
 *   <li>A command <em>must</em> have a non-{@code null} initial state before registration. If
 *       {@link CommandInterface#getCurrentState()} returns {@code null}, {@link #register} will
 *       throw a {@link RuntimeException}. Always call {@code setCurrentState(...)} in the command's
 *       constructor before registering.
 *   <li>Each command should be registered exactly once, usually inside {@code
 *       RobotContainer#configureBindings()} or similar setup methods.
 * </ul>
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * // In RobotContainer:
 * CommandRegisterer.register(new IntakeCommand(controller));
 * CommandRegisterer.register(new ShooterCommand(controller));
 * }</pre>
 *
 * @see CommandRunner
 * @see CommandInterface
 */
public class CommandRegisterer {

  /** Shared list of all registered commands. Populated at startup, read every cycle. */
  private static final List<CommandInterface<?>> commands = new ArrayList<>();

  /**
   * Registers a command with the global registry, making it eligible for execution by {@link
   * CommandRunner} each robot cycle.
   *
   * <p>The command must have a non-{@code null} current state at the time of registration. If it
   * does not, a {@link RuntimeException} is thrown with the message {@code "Set Initial State"}.
   * This guard exists to catch commands whose constructors forgot to call {@code
   * setCurrentState(...)}.
   *
   * @param command the command to register; must have an initial state set
   * @throws RuntimeException if {@code command.getCurrentState()} is {@code null}
   */
  public static void register(CommandInterface<?> command) {
    if (command.getCurrentState() == null) throw new RuntimeException("Set Initial State");
    commands.add(command);
  }

  /**
   * Returns an immutable snapshot of all currently registered commands.
   *
   * <p>Used by {@link CommandRunner#run()} each cycle to obtain the list of commands to execute.
   * The returned list is a copy — modifications to the original registry after this call are not
   * reflected.
   *
   * @return an unmodifiable list of all registered commands; never {@code null}
   */
  public static List<CommandInterface<?>> getCommands() {
    return List.copyOf(commands);
  }
}
