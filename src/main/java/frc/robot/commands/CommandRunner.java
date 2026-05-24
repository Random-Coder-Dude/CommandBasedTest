package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/** Executes all registered commands by running their action pipelines. */
public class CommandRunner {

  /** Runs all registered commands once. */
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

    String base = "Command/" + command.getName();

    Logger.recordOutput(base + "/State/Before", state);

    List<Action<S>> actionList = command.getActions();

    Comparator<Action<S>> comparator =
        Comparator.comparingInt((Action<S> a) -> a.getPriority())
            .reversed()
            .thenComparing(Action::getName);

    // Snapshot original order (debug insight)
    Logger.recordOutput(
        base + "/Telemetry/ActionOrder/Raw",
        actionList.stream().map(Action::getName).toArray(String[]::new));

    actionList.sort(comparator);

    // Snapshot sorted order (debug insight)
    Logger.recordOutput(
        base + "/Telemetry/ActionOrder/Sorted",
        actionList.stream().map(Action::getName).toArray(String[]::new));

    Set<SubsystemBase> subsystemsUsed = new HashSet<>();

    int index = 0;

    for (Action<S> action : actionList) {

      String aBase = base + "/Action/" + action.getName();

      Logger.recordOutput(aBase + "/Index", index);
      Logger.recordOutput(aBase + "/Priority", action.getPriority());
      Logger.recordOutput(aBase + "/State/Input", state);
      Logger.recordOutput(
          aBase + "/SubsystemRequirements",
          action.getSubsystemRequirements().stream()
              .map(SubsystemBase::getName)
              .toArray(String[]::new));

      boolean stateAllowed = action.canRun(state);

      boolean conflict =
          !java.util.Collections.disjoint(subsystemsUsed, action.getSubsystemRequirements());

      Logger.recordOutput(aBase + "/CanRun", stateAllowed);
      Logger.recordOutput(aBase + "/Conflict", conflict);
      Logger.recordOutput(aBase + "/Eligible", stateAllowed && !conflict);

      if (stateAllowed && !conflict) {

        Logger.recordOutput(aBase + "/Decision", "RUN");

        Logger.recordOutput(
            aBase + "/Subsystems/Claimed",
            action.getSubsystemRequirements().stream()
                .map(SubsystemBase::getName)
                .toArray(String[]::new));

        subsystemsUsed.addAll(action.getSubsystemRequirements());

        S oldState = state;
        state = action.run(state);

        Logger.recordOutput(aBase + "/State/Before", oldState);
        Logger.recordOutput(aBase + "/State/After", state);
        Logger.recordOutput(aBase + "/State/Changed", !oldState.equals(state));

        actionsRun++;

        Logger.recordOutput(aBase + "/Result", action.getName() + ": " + oldState + " -> " + state);

      } else {

        Logger.recordOutput(aBase + "/Decision", "SKIP");

        String reason =
            !stateAllowed ? "STATE_MISMATCH" : conflict ? "SUBSYSTEM_CONFLICT" : "UNKNOWN";

        Logger.recordOutput(aBase + "/SkipReason", reason);

        actionsSkipped++;
      }

      index++;
    }

    command.setCurrentState(state);

    Logger.recordOutput(base + "/State/After", state);
    Logger.recordOutput(base + "/Metrics/ActionsRun", actionsRun);
    Logger.recordOutput(base + "/Metrics/ActionsSkipped", actionsSkipped);
    Logger.recordOutput(base + "/Metrics/TotalActions", actionList.size());
    Logger.recordOutput(
        base + "/Metrics/SubsystemsLocked",
        subsystemsUsed.stream().map(SubsystemBase::getName).toArray(String[]::new));
  }
}
