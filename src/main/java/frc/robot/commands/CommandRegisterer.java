package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

/** Global registry for all commands in the system. */
public class CommandRegisterer {

  private static final List<CommandInterface<?>> commands = new ArrayList<>();

  /**
   * Registers a command.
   *
   * @param command command instance
   */
  public static void register(CommandInterface<?> command) {
    if (command.getCurrentState() == null) throw new RuntimeException("Set Initial State");
    if (command.getPriority() <= 0) throw new RuntimeException("Set a Priority");
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
