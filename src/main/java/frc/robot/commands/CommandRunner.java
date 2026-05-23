package frc.robot.commands;

import java.util.List;

/**
 * Executes all registered commands by running their action pipelines.
 */
public class CommandRunner {

  /**
   * Runs all registered commands once.
   */
  public static void run() {
    List<CommandInterface<?>> commands = CommandRegisterer.getCommands();

    for (CommandInterface<?> command : commands) {
      runCommand(command);
    }
  }

  /**
   * Executes a single command's action chain.
   *
   * @param command command to execute
   * @param <S> state type
   */
  private static <S extends Enum<S>> void runCommand(CommandInterface<S> command) {

    S state = command.getCurrentState();

    for (Action<S> action : command.getActions()) {
      if (action.canRun(state)) {
        state = action.run(state);
      }
    }

    command.setCurrentState(state);
  }
}