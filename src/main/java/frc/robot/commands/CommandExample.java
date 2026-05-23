package frc.robot.commands;

public class CommandExample extends CommandBase<CommandExample.State>{

    public CommandExample() {
        addAction(this::switchToRunning);
    }

    public enum State {
        IDLE,
        RUNNING,
        DONE
    }

    public State switchToRunning(State currentState) {
        return State.RUNNING;
    }
}
