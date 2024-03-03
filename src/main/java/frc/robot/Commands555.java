package frc.robot;

import animation2.CelebrationAnimation;
import animation2.FlashAnimation;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.Limelight;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Commands555 {
  private static double initTurnAngle;

  /**
   * Drive to a robot-relative point given a Translation2d & target Rotation2d.
   *
   * @param targetTranslation Field-relative Translation2d to drive the robot to.
   * @param theta Target angle for end position.
   */
  public static Command driveToRobotRelativePoint(
      Translation2d targetTranslation, Rotation2d theta) {
    Pose2d currentRobotPosition = RobotContainer.drivetrain.getSwerveDrive().getPose();
    Rotation2d currentOdometryHeading =
        RobotContainer.drivetrain.getSwerveDrive().getOdometryHeading();

    Translation2d targetTranslation2d =
        currentRobotPosition
            .getTranslation()
            .plus(targetTranslation.rotateBy(currentOdometryHeading));
    Pose2d targetPose = new Pose2d(targetTranslation2d.getX(), targetTranslation2d.getY(), theta);

    return AutoBuilder.pathfindToPose(
        targetPose,
        AutoConstants.PATH_CONSTRAINTS,
        AutoConstants.GOAL_END_VELOCITY,
        AutoConstants.ROTATION_DELAY_DISTANCE);
  }

  // Zero gyro
  public static Command zeroGyro() {
    return Commands.runOnce(RobotContainer.drivetrain.getSwerveDrive()::zeroGyro);
  }

  /**
   * Drive to a field-relative point given a targetPose
   *
   * @param targetPose field-relative pose2d to drive the robot to.
   */
  public static Command driveToFieldRelativePoint(Pose2d targetPose) {
    return AutoBuilder.pathfindToPose(
        targetPose,
        AutoConstants.PATH_CONSTRAINTS,
        AutoConstants.GOAL_END_VELOCITY,
        AutoConstants.ROTATION_DELAY_DISTANCE);
  }

  /*
   * - - - - - - - - - -
   * Intake Commands
   * - - - - - - - - - -
   */
  /**
   * runs the intake until the beam break sensor is broken, and then stops
   *
   * @return A
   */
  public static Command loadNote() {
    Command intakeAndTransport = Commands.parallel(Commands555.intake(), Commands555.transport());
    return intakeAndTransport
        .withName("intake in")
        .until(() -> {
          if (RobotContainer.shooter.isNoteInTransport() == true) {
            System.out.println("<3");
          }
          
          return RobotContainer.shooter.isNoteInTransport();})
        .finallyDo(() -> {
          RobotContainer.intake.stop();
          RobotContainer.shooter.stopTransport();
        });
        
  }

  public static Command intake() {
    return Commands.run(RobotContainer.intake::in);
  }

  public static Command reverseIntake() {
    return Commands.runOnce(RobotContainer.intake::out, RobotContainer.intake)
        .withName("intake out");
  }

  public static Command stopIntake() {
    return Commands.runOnce(RobotContainer.intake::stop, RobotContainer.intake)
        .withName("intake stop");
  }

  // /**
  //  * Robot relative or field relative depending on isFieldRelative. Input angle MUST be between 0
  // and 360 degrees
  //  * @param angle
  //  * @return a command
  //  */

  /***
   *
   * @param rot a field relative Rotation2d supplier for the target angle
   * @param lockDrive should translational motion be locked during the command
   * @return a command that will lock angular control in favor of an angle that is provided
   *
   */
  public static Command alignToAngleFieldRelative(Supplier<Rotation2d> rot, boolean lockDrive) {
    Drivetrain drive = RobotContainer.drivetrain;
    return Commands.run(
            () -> {
              double thetaSpeed =
                  drive
                      .getSwerveDrive()
                      .getSwerveController()
                      .headingCalculate(
                          drive.getWrappedRotation().getRadians(), rot.get().getRadians());

              double xSpeed = 0;
              double ySpeed = 0;

              if (!lockDrive) {
                ySpeed =
                    -MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), 0.05)
                        * DriveConstants.MAX_SPEED;
                xSpeed =
                    -MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.05)
                        * DriveConstants.MAX_SPEED;
              }

              RobotContainer.drivetrain.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed);
              // RobotContainer.drivetrain.drive(targetTranslation, thetaSpeed);
            },
            RobotContainer.drivetrain)
        .until(
            () -> {
              return Math.abs(
                      RobotContainer.drivetrain.getWrappedRotation().getDegrees()
                          - rot.get().getDegrees())
                  < DriveConstants.ANGLE_DEADBAND;
            });
  }

  public static Command alignContinuousFieldRelative(Supplier<Rotation2d> rot, boolean lockDrive) {
    Drivetrain drive = RobotContainer.drivetrain;
    return Commands.run(
            () -> {
              double thetaSpeed =
                  drive
                      .getSwerveDrive()
                      .getSwerveController()
                      .headingCalculate(
                          drive.getWrappedRotation().getRadians(), rot.get().getRadians());

              double xSpeed = 0;
              double ySpeed = 0;

              if (!lockDrive) {
                ySpeed =
                    -MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), 0.05)
                        * DriveConstants.MAX_SPEED;
                xSpeed =
                    -MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.05)
                        * DriveConstants.MAX_SPEED;
              }

              RobotContainer.drivetrain.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed);
              // RobotContainer.drivetrain.drive(targetTranslation, thetaSpeed);
            },
            RobotContainer.drivetrain);
  }

  /***
   *
   * @param rot a robot relative Rotation2d supplier for the target angle
   * @param lockDrive should translational motion be locked during the command
   * @return a command that will lock angular control in favor of an angle that is provided
   *
   */
  public static Command alignToAngleRobotRelative(Supplier<Rotation2d> rot, boolean lockDrive) {
    return alignToAngleFieldRelative(
            () -> {
              return Rotation2d.fromDegrees((initTurnAngle + rot.get().getDegrees()) % 360);
            },
            lockDrive)
        .beforeStarting(
            () -> {
              initTurnAngle = RobotContainer.drivetrain.getWrappedRotation().getDegrees();
            });
  }

  public static Command alignToAngleRobotRelativeContinuous(Supplier<Rotation2d> rot, boolean lockDrive) {
    return alignContinuousFieldRelative(
            () -> {
              return Rotation2d.fromDegrees((RobotContainer.drivetrain.getWrappedRotation().getDegrees() + rot.get().getDegrees()) % 360);
            },
            lockDrive);
  }

  /**
   * @param angle the target angle in field space
   * @param lockDrive should translational motion be locked
   * @return
   */
  public static Command goToAngleFieldRelative(Rotation2d angle, boolean lockDrive) {
    return alignToAngleFieldRelative(
        () -> {
          return angle;
        },
        lockDrive);
  }

  /**
   * @param angle the target angle in robot space
   * @param lockDrive should translational motion be locked
   * @return
   */
  public static Command goToAngleRobotRelative(Rotation2d angle, boolean lockDrive) {
    return alignToAngleRobotRelative(
        () -> {
          return angle;
        },
        lockDrive);
  }

  /**
   * @param camera the limelight to use
   * @return a command that will align the robot to the target from the current limelight Will be
   *     canceled if the limelight loses its target
   */
  public static Command 
  alignToLimelightTarget(Limelight camera) {
    // TODO: needs to use both limelights
    // Rotation2d targetAngle = Rotation2d.fromDegrees(-camera.getObjectTX());
    return ifHasTarget(
            alignToAngleRobotRelativeContinuous(
                () -> {
                  Rotation2d targetAngle = Rotation2d.fromDegrees(-camera.getObjectTX());
                  return targetAngle;
                },
                false),
            camera)
        .finallyDo(
            () -> {
              RobotContainer.drivetrain.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
            }); // TODO should we lock drive?
  }

  public static Command scoreMode() {
    return alignToLimelightTarget(RobotContainer.shooterLimelight);
  }

  /**
   * A command decorator to cancel a command if the limelight loses its target
   *
   * @param cmd the command to be decorated
   * @param limey the limelight to be targeted
   * @return
   */
  public static Command ifHasTarget(Command cmd, Limelight limey) {
    return cmd.onlyWhile(() -> limey.hasValidTarget());
  }

  // TODO: Look over this
  /***
   * @param limey
   * @return A command that drives to the currently targeted april tag
   */
  // public static Command driveToAprilTag(Limelight limey) {
  //     double[] aprilTagPoseArray = LimelightHelpers.getLimelightNTDoubleArray(limey.getName(),
  // "targetpose_robotspace");
  //     Pose2d aprilTagPose = new Pose2d(new Translation2d(aprilTagPoseArray[0],
  // aprilTagPoseArray[1]), new Rotation2d(aprilTagPoseArray[5]));
  //     Pose2d targetPose = aprilTagPose.relativeTo(DriveConstants.EDGE_OF_DRIVEBASE); //TODO does
  // this work the way I think it does
  //     return driveToFieldRelativePoint(targetPose);

  // }

  /***
   * Enables field relative mode
   * @return a command that enables field relative
   */
  public static Command enableFieldRelative() {
    return Commands.runOnce(RobotContainer.drivetrain::enableFieldRelative);
  }

  /**
   * @return a command to disable field relative control
   */
  public static Command disableFieldRelative() {
    return Commands.runOnce(RobotContainer.drivetrain::disableFieldRelative);
  }

  /*
   * - - - - - - - - - -
   * Sprocket Commands
   * - - - - - - - - - -
   */
  // public static Command goUp() {
  //     return Commands.runOnce(RobotContainer.sprocket::goUp,
  // RobotContainer.sprocket).withName("sprocket up");
  // }

  // public static Command goDown() {
  //     return Commands.runOnce(RobotContainer.sprocket::goDown,
  // RobotContainer.sprocket).withName("sprocket down");
  // }

  public static Command stopSprocket() {
    return Commands.runOnce(RobotContainer.sprocket::stop, RobotContainer.sprocket)
        .withName("sprocket stop");
  }

  /**
   * Angle in degrees
   *
   * @param angle
   * @return
   */
  public static Command setSprocketAngle(double angle) {
    return Commands.runOnce(
        () -> RobotContainer.sprocket.setPosition(Rotation2d.fromDegrees(angle)));
  }

  /*
   * - - - - - - - - - -
   * Shooter Commands
   * - - - - - - - - - -
   */
  public static Command shootSpeaker() {
    return Commands.runOnce(RobotContainer.shooter::shootSpeaker, RobotContainer.shooter)
        .withName("shoot speaker");
  }

  public static Command shootAmp() {
    return Commands.runOnce(RobotContainer.shooter::shootAmp, RobotContainer.shooter)
        .withName("shoot amp");
  }

  public static Command stopShooter() {
    return Commands.runOnce(RobotContainer.shooter::stop, RobotContainer.shooter)
        .withName("shooter stop");
  }

  public static Command reverseShooter() {
    return Commands.runOnce(RobotContainer.shooter::reverseShooter, RobotContainer.shooter)
        .withName("shooter reverse");
  }

  public static Command shootVelocity(double velocity) {
    return Commands.runOnce(
        () -> {
          RobotContainer.shooter.shootVelocity(5000);
        });
  }

  public Command shootSequence(double angle, double velocity) {
    return Commands.sequence(
        shootVelocity(velocity),
        setSprocketAngle(angle),
        waitUntil(
            () -> {
              return RobotContainer.shooter.isAtSetpoint(velocity)
                  && RobotContainer.sprocket.isAtAngle();
            }),
        transport(),
        waitForTime(3.5),
        setSprocketAngle(ArmConstants.ENCODER_MIN_ANGLE),
        shootVelocity(0));
  }

  public Command waitUntil(BooleanSupplier condition) {
    return new Command() {
      @Override
      public boolean isFinished() {
        return condition.getAsBoolean();
      }
    };
  }

  public Command waitForTime(double seconds) {
    return Commands.run(() -> {}).withTimeout(seconds);
  }

  public static Command transport() {
    return Commands.run(
        () -> {
          RobotContainer.shooter.transportStart();
        });
  }

  public static Command scoreAmp() {
    return Commands.sequence(
        alignToLimelightTarget(RobotContainer.shooterLimelight),
        setSprocketAngle(ArmConstants.AMP_SCORE_ANGLE),
        shootAmp(),
        setSprocketAngle(ArmConstants.ENCODER_MIN_ANGLE));
  }

  public static Command scoreSpeaker() {
    return Commands.sequence(
        alignToLimelightTarget(RobotContainer.shooterLimelight),
        setSprocketAngle(ArmConstants.SPEAKER_SCORE_ANGLE),
        shootSpeaker(),
        setSprocketAngle(ArmConstants.ENCODER_MIN_ANGLE));
  }

  public static Command receiveHumanPlayerNote() {
    return Commands.sequence(
        alignToLimelightTarget(RobotContainer.shooterLimelight),
        setSprocketAngle(ArmConstants.SPEAKER_SCORE_ANGLE),
        reverseShooter(),
        setSprocketAngle(ArmConstants.ENCODER_MIN_ANGLE));
  }

  public static Command signalAmp() {
    return Commands.runOnce(
        () -> {
          RobotContainer.led.add(new FlashAnimation(Color.kOrange));
        });
  }

  public static Command signalCoop() {
    return Commands.runOnce(
        () -> {
          RobotContainer.led.add(new FlashAnimation(Color.kBlue));
        });
  }

  // LED bits
  public static Command celebrate() {
    return Commands.runOnce(
        () -> {
          RobotContainer.led.add(new CelebrationAnimation());
        });
  }

  public static Command ampItUp() {
    return Commands.runOnce(
        () -> {
          RobotContainer.led.add(new FlashAnimation(Color.kYellow));
        });
  }

  public static Command cooperatition() {
    return Commands.runOnce(
        () -> {
          RobotContainer.led.add(new FlashAnimation(Color.kBlueViolet));
        });
  }

  // ***********************CLIMBER COMMANDS*************************//
  public static Command climberUp() {
    return Commands.runOnce(
        () -> {
          RobotContainer.climbers.up();
        });
  }

  public static Command climberDown() {
    return Commands.runOnce(
        () -> {
          RobotContainer.climbers.down();
        });
  }
  public static Command climberStop() {
    return Commands.runOnce(() -> {
      RobotContainer.climbers.stop();
    });
  }
}
