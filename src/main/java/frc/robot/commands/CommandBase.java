package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public abstract class CommandBase<S extends Enum<S>> implements CommandInterface<S> {
    /**
     * Current state of the Runner
     */
    protected S state;
    /**
     * List of actions the Runner will cycle between
     */
    protected final List<Function<S,S>> actions = new ArrayList<>();

    @Override
    public void setCurrentState(S newState) {
        state = newState;
    }

    @Override 
    public S getCurrentState() {
        return state;
    }

    @Override
    public List<Function<S,S>> getActions() {
        return actions;
    }

    /**
     * Adds the function specificed to the Actions list
     * @param action {@code addAction(this::functionName)} 
     */
    public void addAction(Function<S,S> action) {
        actions.add(action);
    }
}