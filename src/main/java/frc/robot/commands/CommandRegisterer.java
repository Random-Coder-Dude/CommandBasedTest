package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

/**
 * Global registry for all commands in the system.
 */
public class CommandRegisterer {

  private static final List<CommandInterface<?>> commands = new ArrayList<>();

  /**
   * Registers a command.
   *
   * @param command command instance
   */
  public static void register(CommandInterface<?> command) {
    commands.add(command);
  }

  /**
   * Returns all registered commands.
   *
   * @return list of commands
   */
  public static List<CommandInterface<?>> getCommands() {
    return List.copyOf(commands);
  }
}