package frc.robot.commands;

import java.util.List;
import java.util.function.Function;

public class CommandRunner {

    public static void run() {

        List<CommandInterface<?>> commands = CommandRegisterer.getCommands();

        for (CommandInterface<?> command : commands) {
            runCommand(command);
        }
    }

    private static <S extends Enum<S>> void runCommand(CommandInterface<S> command) {

        S state = command.getCurrentState();

        for (Function<S, S> action : command.getActions()) {
            state = action.apply(state);
        }

        command.setCurrentState(state);
    }
}