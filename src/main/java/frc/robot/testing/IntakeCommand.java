package frc.robot.testing;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CommandBase;

/**
 * Example production-style FSM using the custom command framework.
 *
 * <p>Controls a hypothetical intake system with controller input.
 */
public class IntakeCommand extends CommandBase<IntakeCommand.State> {

  private final CommandXboxController controller;

  public IntakeCommand(CommandXboxController controller) {
    this.controller = controller;

    setCurrentState(State.OFF);

    addAction(
        "InputHandler",
        this::handleInput,
        State.OFF,
        State.INTAKING,
        State.HOLDING,
        State.OUTTAKING);
    addAction("IntakeLogic", this::runIntakeLogic, State.INTAKING);
    addAction("HoldLogic", this::runHoldLogic, State.HOLDING);
    addAction("OuttakeLogic", this::runOuttakeLogic, State.OUTTAKING);
  }

  /** FSM states for intake control. */
  public enum State {
    OFF,
    INTAKING,
    HOLDING,
    OUTTAKING
  }

  /** Handles controller input and determines state transitions. */
  private State handleInput(State state) {

    if (controller.getHID().getAButton()) {
      return State.INTAKING;
    }

    if (controller.getHID().getBButton()) {
      return State.OUTTAKING;
    }

    return State.HOLDING;
  }

  /** Intake running behavior. */
  private State runIntakeLogic(State state) {
    System.out.println("[Intake] INTAKING motors forward");
    // intakeMotor.set(1.0);

    return state;
  }

  /** Holding behavior (stop motors, maintain position). */
  private State runHoldLogic(State state) {
    System.out.println("[Intake] HOLDING position");
    // intakeMotor.set(0.1);

    return state;
  }

  /** Outtake behavior. */
  private State runOuttakeLogic(State state) {
    System.out.println("[Intake] OUTTAKING motors reverse");
    // intakeMotor.set(-1.0);

    return state;
  }
}
