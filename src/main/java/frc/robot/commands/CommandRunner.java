package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * Execution engine for the custom FSM command framework.
 *
 * <p>{@code CommandRunner} is a static utility class called once per robot cycle (from {@code
 * Robot#robotPeriodic()}). It iterates every command registered with {@link CommandRegisterer},
 * runs each command's eligible action pipeline, and commits the resulting state back to the
 * command.
 *
 * <h2>Per-Command Execution Pipeline</h2>
 *
 * <p>For each registered command, the runner performs the following steps:
 *
 * <ol>
 *   <li><b>Snapshot state:</b> Reads {@link CommandInterface#getCurrentState()} as the starting
 *       state for this cycle.
 *   <li><b>Sort actions:</b> Sorts the command's action list by priority (descending), then
 *       alphabetically by name for deterministic tie-breaking.
 *   <li><b>Evaluate actions:</b> For each action in sorted order:
 *       <ul>
 *         <li>Checks {@link Action#canRun(Enum)} — skips with reason {@code STATE_MISMATCH} if the
 *             current state is not in the action's requirements.
 *         <li>Checks for subsystem conflicts — skips with reason {@code SUBSYSTEM_CONFLICT} if a
 *             previously-run action already claimed one of this action's required subsystems.
 *         <li>If eligible: runs the action via {@link Action#run(Enum)}, threads the returned state
 *             forward, and claims the action's subsystems for the rest of the cycle.
 *       </ul>
 *   <li><b>Commit state:</b> Calls {@link CommandInterface#setCurrentState} with the final state
 *       produced by the action chain.
 * </ol>
 *
 * <h2>AdvantageKit Telemetry</h2>
 *
 * <p>Every step of the pipeline is logged to AdvantageKit under the key prefix {@code
 * Command/<CommandName>/}. Useful log keys include:
 *
 * <ul>
 *   <li>{@code Command/<name>/State/Before} — state entering the cycle
 *   <li>{@code Command/<name>/State/After} — state exiting the cycle
 *   <li>{@code Command/<name>/Action/<actionName>/Decision} — {@code "RUN"} or {@code "SKIP"}
 *   <li>{@code Command/<name>/Action/<actionName>/SkipReason} — {@code STATE_MISMATCH} or {@code
 *       SUBSYSTEM_CONFLICT}
 *   <li>{@code Command/<name>/Metrics/ActionsRun} — count of actions executed this cycle
 *   <li>{@code Command/<name>/Metrics/ActionsSkipped} — count of actions skipped this cycle
 * </ul>
 *
 * <h2>Calling Convention</h2>
 *
 * <p>{@link #run()} is called once per cycle from {@code Robot#robotPeriodic()}, after {@code
 * CommandScheduler.getInstance().run()}. It must not be called more than once per cycle.
 *
 * @see CommandRegisterer
 * @see CommandInterface
 * @see Action
 */
public class CommandRunner {

  /**
   * Executes one cycle of all registered commands.
   *
   * <p>Retrieves the current command list from {@link CommandRegisterer#getCommands()} and
   * delegates to {@link #runCommand(CommandInterface)} for each entry. This method is intended to
   * be called exactly once per robot cycle from {@code Robot#robotPeriodic()}.
   */
  public static void run() {
    List<CommandInterface<?>> commands = CommandRegisterer.getCommands();
    for (CommandInterface<?> command : commands) {
      runCommand(command);
    }
  }

  /**
   * Executes a single command's full action pipeline for one robot cycle.
   *
   * <p>This method handles sorting, eligibility checking, subsystem conflict detection, action
   * execution, state threading, and AdvantageKit telemetry. See class-level documentation for the
   * full step-by-step description.
   *
   * <p>This method uses a generic type parameter {@code S} to maintain type safety when reading and
   * writing state, even though commands are stored as raw {@code CommandInterface<?>} in the
   * registry.
   *
   * @param command the command to execute this cycle
   * @param <S> the enum state type of the command
   * @throws RuntimeException if state is read as null
   */
  private static <S extends Enum<S>> void runCommand(CommandInterface<S> command) {

    S state = command.getCurrentState();
    if (state == null) throw new RuntimeException("State Read as Null");

    int actionsRun = 0;
    int actionsSkipped = 0;

    String base = "Command/" + command.getName();

    Logger.recordOutput(base + "/State/Before", state);

    List<Action<S>> actionList = new ArrayList<>(command.getActions());

    Comparator<Action<S>> comparator =
        Comparator.comparingInt((Action<S> a) -> a.getPriority())
            .reversed()
            .thenComparing(actionList::indexOf);

    // Snapshot original order (debug insight)
    Logger.recordOutput(
        base + "/Telemetry/ActionOrder/Raw",
        actionList.stream().map(Action::getName).toArray(String[]::new));

    actionList.sort(comparator);

    // Snapshot sorted order (debug insight)
    Logger.recordOutput(
        base + "/Telemetry/ActionOrder/Sorted",
        actionList.stream().map(Action::getName).toArray(String[]::new));

    Set<Subsystem> subsystemsUsed = new HashSet<>();

    for (Action<S> action : actionList) {

      String aBase = base + "/Action/" + action.getName();

      Logger.recordOutput(aBase + "/Priority", action.getPriority());
      Logger.recordOutput(aBase + "/State/Input", state);
      Logger.recordOutput(
          aBase + "/SubsystemRequirements",
          action.getSubsystemRequirements().stream()
              .map(Subsystem::getName)
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
                .map(Subsystem::getName)
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
    }

    command.setCurrentState(state);

    Logger.recordOutput(base + "/State/After", state);
    Logger.recordOutput(base + "/Metrics/ActionsRun", actionsRun);
    Logger.recordOutput(base + "/Metrics/ActionsSkipped", actionsSkipped);
    Logger.recordOutput(base + "/Metrics/TotalActions", actionList.size());
    Logger.recordOutput(
        base + "/Metrics/SubsystemsLocked",
        subsystemsUsed.stream().map(Subsystem::getName).toArray(String[]::new));
  }
}
