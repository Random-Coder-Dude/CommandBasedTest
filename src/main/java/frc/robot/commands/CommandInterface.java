package frc.robot.commands;

import java.util.List;
import java.util.function.Function;

public interface CommandInterface<S extends Enum<S>>{
    /**
     * Gets the Current State the Command is in
     */
    S getCurrentState();
    /**
     * Sets the Current State the Command is in
     */
    void setCurrentState(S state);
    /**
     * Gets the list of Actions the Runner will cycle between
     */
    List<Function<S,S>> getActions();
}
