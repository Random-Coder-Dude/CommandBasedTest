package frc.robot.Swerve.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.CommandBase;
import frc.robot.Swerve.Subsystem.SwerveSubsystem;
import java.util.function.BooleanSupplier;

public class SwerveSysId extends CommandBase<SwerveSysId.State> {
  private BooleanSupplier runNext;
  private Command quasistaticFront;
  private Command quasistaticBack;
  private Command dynamicFront;
  private Command dynamicBack;
  private State lastState = State.NONE;

  public enum State {
    NONE,
    QUASISTATIC_FRONT,
    DYNAMIC_FRONT,
    QUASISTATIC_BACK,
    DYNAMIC_BACK,
    DRIVE_WHEEL_CHARACTERIZATION
  }

  public SwerveSysId(SwerveSubsystem drivetrain, BooleanSupplier runNext) {
    this.runNext = runNext;

    quasistaticFront = drivetrain.sysIdQuasistatic(Direction.kForward);
    quasistaticBack = drivetrain.sysIdQuasistatic(Direction.kReverse);
    dynamicFront = drivetrain.sysIdDynamic(Direction.kForward);
    dynamicBack = drivetrain.sysIdDynamic(Direction.kReverse);

    setCurrentState(State.NONE);

    addAction("Input Handler", this::handleInputs).withPriority(1000);
    addAction("Quasistatic Front", this::handleQuasistaticFront, State.QUASISTATIC_FRONT).withSubsystems(drivetrain).withPriority(10);
    addAction("Command Handler", this::)
  }

  private State handleInputs(State state) {
    if (runNext.getAsBoolean()) {
      return switch (state) {
        case NONE -> State.QUASISTATIC_FRONT;
        case QUASISTATIC_FRONT -> State.DYNAMIC_FRONT;
        case DYNAMIC_FRONT -> State.QUASISTATIC_BACK;
        case QUASISTATIC_BACK -> State.DYNAMIC_BACK;
        case DYNAMIC_BACK -> State.DRIVE_WHEEL_CHARACTERIZATION;
        case DRIVE_WHEEL_CHARACTERIZATION -> State.NONE;
      };
    }
    return state;
  }

  private State handleQuasistaticFront(State state) {
    if (!quasistaticFront.isScheduled()) {
        quasistaticFront.schedule();
    }

    return state;
  }

  private State handleSwitches(State state) {
    if (state != lastState) {
        quasistaticFront.cancel();
        quasistaticBack.cancel();
        dynamicFront.cancel();
        dynamicBack.cancel();
    }

    
  }
}
