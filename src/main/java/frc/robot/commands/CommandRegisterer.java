package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

public class CommandRegisterer {
    private static final List<CommandInterface<?>> commands = new ArrayList<>();

    /**
     * Registers a Command instance for a Runner
     * @param command Command to register
     */
    public static void register(CommandInterface<?> command) {
        commands.add(command);
    }

    /**
     * Returns all registered commands.
     */
    public static List<CommandInterface<?>> getCommands() {
        return commands;
    }
}
