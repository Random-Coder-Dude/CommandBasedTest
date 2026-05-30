package frc.robot.Swerve.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Commands.CommandBase;
import frc.robot.Constants;
import frc.robot.Swerve.Generated.TunerConstants;
import frc.robot.Swerve.Subsystem.SwerveSubsystem;
import frc.robot.Utils.CougarUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SwerveCommand extends CommandBase<SwerveCommand.State> {

  private SwerveSubsystem drivetrain;
  private DoubleSupplier horizontalSupplier;
  private DoubleSupplier verticalSupplier;
  private DoubleSupplier rotationSupplier;
  private BooleanSupplier X;
  private BooleanSupplier robotRelative;
  private BooleanSupplier align;
  private BooleanSupplier isSlowed;
  private BooleanSupplier isFast;
  private BooleanSupplier resetHeadingSupplier;
  private Supplier<Pose2d> targetPose;

  private double speedLimiter;
  private ChassisSpeeds driveChassisSpeeds = new ChassisSpeeds();
  private SlewRateLimiter rotationLimiter;
  private double prevHorizontal = 0;
  private double prevVertical = 0;
  private static final double maxVelocityChange = 13 * Constants.kLoopTime;
  private static final double translationP = 2;
  private static final double rotationalP = 2;

  public enum State {
    X, // X Mode (Anti Movement)
    FIELD_RELATIVE, // Standard Driving
    ROBOT_RELATIVE, // Robot Front is Front
    ALIGNING // Aligning to Target Pose
  }

  public SwerveCommand(
      SwerveSubsystem drivetrain,
      DoubleSupplier horizontalSupplier,
      DoubleSupplier verticalSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier X,
      BooleanSupplier robotRelative,
      BooleanSupplier align,
      BooleanSupplier isSlowed,
      BooleanSupplier isFast,
      BooleanSupplier resetHeadingSupplier,
      Supplier<Pose2d> targetPose) {
    this.drivetrain = drivetrain;
    this.horizontalSupplier = horizontalSupplier;
    this.verticalSupplier = verticalSupplier;
    this.rotationSupplier = rotationSupplier;
    this.X = X;
    this.robotRelative = robotRelative;
    this.align = align;
    this.isSlowed = isSlowed;
    this.isFast = isFast;
    this.resetHeadingSupplier = resetHeadingSupplier;
    this.targetPose = targetPose;

    this.speedLimiter = 0;

    prevHorizontal = prevVertical = 0;
    rotationLimiter = new SlewRateLimiter(3, -3, 0);

    setCurrentState(State.FIELD_RELATIVE);

    addAction("Internals", this::handleInternals).withPriority(1000);
    addAction("Input Handler", this::handleInputs).withPriority(1000);
    addAction("Aligning", this::aligning, State.ALIGNING)
        .withSubsystems(drivetrain)
        .withPriority(100);
    addAction("X Mode", this::xMode, State.X).withSubsystems(drivetrain).withPriority(90);
    addAction("Field Drive", this::fieldDrive, State.FIELD_RELATIVE)
        .withSubsystems(drivetrain)
        .withPriority(10);
    addAction("Robot Drive", this::robotDrive, State.ROBOT_RELATIVE)
        .withSubsystems(drivetrain)
        .withPriority(9);
  }

  public State handleInternals(State state) {

    if (resetHeadingSupplier.getAsBoolean()) {
      if (DriverStation.getAlliance().get() == Alliance.Red)
        drivetrain.resetRotation(Rotation2d.kPi);
      else drivetrain.resetRotation(Rotation2d.kZero);
    }

    if (isSlowed.getAsBoolean()) {
      speedLimiter = 0.3;
    } else if (isFast.getAsBoolean()) {
      speedLimiter = 1.0;
    } else {
      speedLimiter = 0.7;
    }

    return state;
  }

  public State handleInputs(State state) {

    if (align.getAsBoolean()) return State.ALIGNING;

    if (X.getAsBoolean()) return State.X;

    if (robotRelative.getAsBoolean()) return State.ROBOT_RELATIVE;

    return State.FIELD_RELATIVE;
  }

  public State aligning(State state) {
    Pose2d robot = drivetrain.getPose();
    Pose2d target = targetPose.get();

    double dx = target.getX() - robot.getX();
    double dy = target.getY() - robot.getY();

    double vx = dx * translationP;
    double vy = dy * translationP;

    double mag = Math.hypot(vx, vy);
    double maxSpeed = TunerConstants.kMaxSpeed;

    if (mag > maxSpeed) {
      vx *= maxSpeed / mag;
      vy *= maxSpeed / mag;
    }

    double angular = (target.getRotation().minus(robot.getRotation()).getRadians()) * rotationalP;

    angular =
        MathUtil.clamp(angular, -TunerConstants.kMaxAngularRate, TunerConstants.kMaxAngularRate);

    driveChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(vx, vy, angular), drivetrain.getShallowRotation());

    drivetrain.drive(driveChassisSpeeds);
    return state;
  }

  public State xMode(State state) {
    drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    return state;
  }

  public State fieldDrive(State state) {
    Logger.recordOutput("Chassis Speed", driveChassisSpeeds);
    calculateChassisSpeeds(true);
    drivetrain.drive(driveChassisSpeeds);
    return state;
  }

  public State robotDrive(State state) {
    calculateChassisSpeeds(false);
    drivetrain.drive(driveChassisSpeeds);
    return state;
  }

  public void calculateChassisSpeeds(boolean fieldRelative) {
    double horizontal = horizontalSupplier.getAsDouble();
    double vertical = verticalSupplier.getAsDouble();
    double vel_hypot = Math.hypot(horizontal, vertical);

    if (CougarUtil.getAlliance() == Alliance.Red && fieldRelative) {
      horizontal *= -1;
      vertical *= -1;
    }

    if (vel_hypot > 0) {
      // normalize using vector scaling
      double velocity = MathUtil.clamp(vel_hypot, 0, 1);
      velocity = MathUtil.applyDeadband(velocity, 0.05);

      // scale unit vector by speed limiter and convert to speed
      horizontal *= velocity / vel_hypot * TunerConstants.kMaxSpeed * speedLimiter;
      vertical *= velocity / vel_hypot * TunerConstants.kMaxSpeed * speedLimiter;
    }

    double ang_deadband = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.05);

    double angular =
        rotationLimiter.calculate(squareNum(ang_deadband) * speedLimiter)
            * TunerConstants.kMaxAngularRate;

    // limit change in translation of the overall robot, based on orbit's slideshow
    {
      double dx = horizontal - prevHorizontal;
      double dy = vertical - prevVertical;
      double dmag = Math.hypot(dx, dy);

      if (dmag > 0) {
        double scale = MathUtil.clamp(dmag, 0, maxVelocityChange) / dmag;
        dx *= scale;
        dy *= scale;
        horizontal = prevHorizontal + dx;
        vertical = prevVertical + dy;
      }

      prevHorizontal = horizontal;
      prevVertical = vertical;
    }

    {
      if (fieldRelative) {
        driveChassisSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vertical, horizontal, angular, drivetrain.getShallowRotation());
      } else {
        driveChassisSpeeds = new ChassisSpeeds(vertical, horizontal, angular);
      }
    }
  }

  private static double squareNum(double num) {
    return Math.signum(num) * Math.pow(num, 2);
  }
}
