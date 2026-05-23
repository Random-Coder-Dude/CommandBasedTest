package frc.robot.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

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

  int actionsRun = 0;
  int actionsSkipped = 0;

  Logger.recordOutput("Command/" + command.getName() + "/State", state);

  for (Action<S> action : command.getActions()) {

    if (action.canRun(state)) {

      S oldState = state;
      state = action.run(state);

      actionsRun++;

      Logger.recordOutput(
          "Command/" + command.getName() + "/ActionRun",
          action.getName()
              + ": " + oldState + " -> " + state
      );

    } else {
      actionsSkipped++;
    }
  }

  command.setCurrentState(state);

  Logger.recordOutput("Command/" + command.getName() + "/State", state);
  Logger.recordOutput("Command/" + command.getName() + "/ActionsRun", actionsRun);
  Logger.recordOutput("Command/" + command.getName() + "/ActionsSkipped", actionsSkipped);
}
}