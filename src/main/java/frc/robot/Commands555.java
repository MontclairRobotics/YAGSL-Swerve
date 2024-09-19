package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.math.Math555;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.DetectionType;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightHelpers;

public class Commands555 {
  private static double initTurnAngle;

  /**
   * Drive to a robot-relative point given a Translation2d & target Rotation2d.
   *
   * @param targetTranslation Field-relative Translation2d to drive the robot to.
   * @param theta             Target angle for end position.
   */
  // public static Command driveToRobotRelativePoint(
  //     Translation2d targetTranslation, Rotation2d theta) {
  //   Pose2d currentRobotPosition = RobotContainer.drivetrain.getSwerveDrive().getPose();
  //   Rotation2d currentOdometryHeading = RobotContainer.drivetrain.getSwerveDrive().getOdometryHeading();

  //   Translation2d targetTranslation2d = currentRobotPosition
  //       .getTranslation()
  //       .plus(targetTranslation.rotateBy(currentOdometryHeading));
  //   Pose2d botPose = new Pose2d(targetTranslation2d.getX(), targetTranslation2d.getY(), theta);

  //   // return AutoBuilder.pathfindToPose(
  //   //     botPose,
  //   //     AutoConstants.PATH_CONSTRAINTS,
  //   //     AutoConstants.GOAL_END_VELOCITY,
  //   //     AutoConstants.ROTATION_DELAY_DISTANCE);
  // }

  // Zero gyro
  public static Command zeroGyro() {
    return Commands.runOnce(() -> {
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        //RobotContainer.drivetrain.getSwerveDrive().zeroGyro();
        RobotContainer.drivetrain.getSwerveDrive().resetOdometry(new Pose2d(RobotContainer.drivetrain.getSwerveDrive().getPose().getTranslation(), Rotation2d.fromDegrees(180)));
      } else {
        RobotContainer.drivetrain.getSwerveDrive().zeroGyro();
      }
    });
  }



  public static Command lockDrive() {
    return Commands.runOnce(() -> {
      RobotContainer.drivetrain.getSwerveDrive().lockPose();
    }, RobotContainer.drivetrain);
  }

  /**
   * Drive to a field-relative point given a botPose
   *
   * @param botPose field-relative pose2d to drive the robot to.
   */
  // public static Command driveToFieldRelativePoint(Pose2d botPose) {
  //   return AutoBuilder.pathfindToPose(
  //       botPose,
  //       AutoConstants.PATH_CONSTRAINTS,
  //       AutoConstants.GOAL_END_VELOCITY,
  //       AutoConstants.ROTATION_DELAY_DISTANCE);
  // }

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
    Command alignSprocket = Commands555.setSprocketAngle(ArmConstants.INTAKE_ANGLE);
    Command intakeAndTransport = Commands.sequence(alignSprocket, Commands.parallel(Commands555.intake(), Commands555.transport(ShooterConstants.TRANSPORT_SPEED)));
    return intakeAndTransport
        .withName("intake in")
        .until(() -> {
          return RobotContainer.shooter.isNoteInTransport();
        })
        .finallyDo(() -> {
          RobotContainer.intake.stop();
          RobotContainer.shooter.stopTransport();
        });

  }

  public static Command loadNoteSource() {
    Command alignSprocket = Commands555.setSprocketAngle(ArmConstants.SOURCE_ANGLE);
    Command intakeAndTransport = Commands.sequence(alignSprocket,
        Commands.parallel(Commands.runOnce(() -> {RobotContainer.shooter.shootVelocity(-ShooterConstants.SOURCE_SPEED);}), Commands555.transport(-ShooterConstants.TRANSPORT_SPEED), Commands555.waitUntil(() -> false)));
    return intakeAndTransport
        .withName("load note source")
        .until(() -> {
          return RobotContainer.shooter.isNoteInTransport();
        })
        .finallyDo(() -> {
          RobotContainer.shooter.stop();
          RobotContainer.shooter.stopTransport();
        });
  }

  public static Command unloadNote() {
    Command alignSprocket = Commands555.setSprocketAngle(ArmConstants.OUTTAKE_ANGLE);
    Command intakeAndTransport = Commands.sequence(alignSprocket,
        Commands.parallel(Commands555.reverseIntake(), Commands.sequence(waitForTime(0.5), Commands555.transport(-ShooterConstants.TRANSPORT_SPEED))));
    return intakeAndTransport
        .withName("unload note")
        // .until(() -> {
        //   return RobotContainer.shooter.isNoteInTransport();
        // })
        .finallyDo(() -> {
          RobotContainer.intake.stop();
          RobotContainer.shooter.stopTransport();
        });

  }


  

  public static Command setChassiSpeeds(ChassisSpeeds speeds) {
    return Commands.run(() -> {
      RobotContainer.drivetrain.setChassisSpeeds(speeds);
    });
  }
  public static Command loadNoteAutoOld() {

    Command driveIntake = Commands.race(Commands555.alignToLimelightTargetWithDrive(RobotContainer.intakeLimelight, DetectionType.NOTE, new Translation2d(1.5, 0)),
        Commands555.intake(), Commands.waitUntil(() -> {
          return RobotContainer.shooter.isNoteInTransport();
        }));
    return Commands.sequence(
        log("CHASING NOOOOOOOOTEEEE"),
        Commands.parallel(Commands555.alignToLimelightTargetWithStop(RobotContainer.intakeLimelight, DetectionType.NOTE),
            Commands555.setSprocketAngle(ArmConstants.INTAKE_ANGLE)),
        log("Done aligning"),
        Commands.waitUntil(() -> {return RobotContainer.sprocket.isAtAngle() && RobotContainer.intakeLimelight.isAligned();}),
        log("Done waiting, hhunting down note"),
        Commands555.transport(ShooterConstants.TRANSPORT_SPEED),
        log("Drive Intaking"),
        driveIntake.withTimeout(1))
        .finallyDo(() -> {
          RobotContainer.intake.stop();
          RobotContainer.shooter.stopTransport();
          System.out.println("Load Note Auto Canceled!");
        });

  }

  public static Command loadNoteAuto() {
    return Commands.sequence(
      log("Loading Note Auto!"),
      Commands.parallel(
        alignToLimelightTargetWithStop(RobotContainer.intakeLimelight, DetectionType.NOTE),
        Commands555.setSprocketAngleWithStop(() -> ArmConstants.INTAKE_ANGLE)
      ),
      log("Done Aligning, beginning intake"),
      Commands555.transport(ShooterConstants.TRANSPORT_SPEED),
      Commands.parallel(
        Commands555.intake(),
        Commands555.alignToLimelightTargetWithDrive(RobotContainer.intakeLimelight, DetectionType.NOTE, new Translation2d(1.5, 0))
      ).until(RobotContainer.shooter::isNoteInTransport).withTimeout(2)
    ).finallyDo(() -> {
      RobotContainer.intake.stop();
      RobotContainer.shooter.stopTransport();
      System.out.println("Load Note Auto Canceled!");
    });
  }

  public static Command driveOneMeter() {
    return Commands.run(RobotContainer.drivetrain::driveOneMeter);
  }

  public static Command intake() {
    return Commands.run(RobotContainer.intake::in);
  }

  public static Command reverseIntake() {
    return Commands.run(RobotContainer.intake::out)
        .withName("intake out");
  }

  public static Command stopIntake() {
    return Commands.runOnce(RobotContainer.intake::stop, RobotContainer.intake)
        .withName("intake stop");
  }

  public static Command ferryNote(double angle) {
    return
    Commands.sequence(
      setSprocketAngle(angle),
      waitUntil(RobotContainer.sprocket::isAtAngle),
      Commands555.shoot(ShooterConstants.FERRY_SHOOT_SPEED, ShooterConstants.FERRY_SHOOT_SPEED, ShooterConstants.TRANSPORT_FERRY_SPEED)
    );
  }
  // /**
  // * Robot relative or field relative depending on isFieldRelative. Input angle
  // MUST be between 0
  // and 360 degrees
  // * @param angle
  // * @return a command
  // */

  /***
   *
   * @param rot       a field relative Rotation2d supplier for the target angle
   * @param lockDrive should translational motion be locked during the command
   * @return a command that will lock angular control in favor of an angle that is
   *         provided
   *
   */
  public static Command alignToAngleFieldRelative(Supplier<Rotation2d> rot, boolean lockDrive) {
    Drivetrain drive = RobotContainer.drivetrain;
    return Commands.run(
        () -> {
          double thetaSpeed = drive
              .getSwerveDrive()
              .getSwerveController()
              .headingCalculate(
                  drive.getWrappedRotation().getRadians(), rot.get().getRadians());

          double xSpeed = 0;
          double ySpeed = 0;

          if (!lockDrive) {
            ySpeed = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), 0.05)
                * DriveConstants.MAX_SPEED;
            xSpeed = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.05)
                * DriveConstants.MAX_SPEED;
          }

          if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)  {
            
             xSpeed *= -1;
             ySpeed *= -1;
          }

          RobotContainer.drivetrain.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed);
          // RobotContainer.drivetrain.drive(targetTranslation, thetaSpeed);
        },
        RobotContainer.drivetrain)
        .until(
            () -> {
              // boolean isAligned = Drivetrain.angleDeadband(
              //     RobotContainer.drivetrain.getWrappedRotation(),
              //         rot.get(), Rotation2d.fromDegrees(DriveConstants.ANGLE_DEADBAND));
              // System.out.println(rot.get().getDegrees());
              // System.out.println(isAligned);
              // System.out.println(RobotContainer.drivetrain.getWrappedRotation().getDegrees());

              return RobotContainer.drivetrain.getSwerveDrive().getSwerveController().thetaController.atSetpoint();
            });
  }

  public static Command alignContinuousFieldRelative(Supplier<Rotation2d> rot, boolean lockDrive) {
    Drivetrain drive = RobotContainer.drivetrain;
    return Commands.run(
        () -> {
          double thetaSpeed = drive
              .getSwerveDrive()
              .getSwerveController()
              .headingCalculate(
                  drive.getWrappedRotation().getRadians(), rot.get().getRadians());

          double xSpeed = 0;
          double ySpeed = 0;

          if (!lockDrive) {
            ySpeed = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), 0.05)
                * DriveConstants.MAX_SPEED;
            xSpeed = -MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.05)
                * DriveConstants.MAX_SPEED;
          }

          if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)  {
           
             xSpeed *= -1;
             ySpeed *= -1;
          }

          RobotContainer.drivetrain.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed);
          // RobotContainer.drivetrain.drive(targetTranslation, thetaSpeed);
        },
        RobotContainer.drivetrain);
  }

    public static Command alignContinuousFieldRelativeWithDrive(Supplier<Rotation2d> rot, Translation2d speeds) {
    Drivetrain drive = RobotContainer.drivetrain;
    return Commands.run(
        () -> {
          double thetaSpeed = drive
              .getSwerveDrive()
              .getSwerveController()
              .headingCalculate(
                  drive.getWrappedRotation().getRadians(), rot.get().getRadians());

          RobotContainer.drivetrain.getSwerveDrive().drive(new Translation2d(speeds.getX(), speeds.getY()), thetaSpeed, false, true, new Translation2d() );
          // RobotContainer.drivetrain.drive(targetTranslation, thetaSpeed);
        },
        RobotContainer.drivetrain);
  }

  /***
   *
   * @param rot       a robot relative Rotation2d supplier for the target angle
   * @param lockDrive should translational motion be locked during the command
   * @return a command that will lock angular control in favor of an angle that is
   *         provided
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
          return Rotation2d.fromDegrees(
              (RobotContainer.drivetrain.getWrappedRotation().getDegrees() + rot.get().getDegrees()) % 360);
        },
        lockDrive);
  }

  public static Command alignToAngleRobotRelativeContinuousWithDrive(Supplier<Rotation2d> rot, Translation2d speeds) {
    return alignContinuousFieldRelativeWithDrive(
        () -> {
          return Rotation2d.fromDegrees(
              (RobotContainer.drivetrain.getWrappedRotation().getDegrees() + rot.get().getDegrees()) % 360);
        },
        speeds);
  }

  public static Command getShooterSysIdCommand(String motors) {
      return new WaitCommand(5)
        .andThen(RobotContainer.shooter.shooterSysId("quasistatic", motors, SysIdRoutine.Direction.kForward))
        .andThen(new WaitCommand(5))
        .andThen(RobotContainer.shooter.shooterSysId("quasistatic", motors, SysIdRoutine.Direction.kReverse))
        .andThen(new WaitCommand(5))
        .andThen(RobotContainer.shooter.shooterSysId("dynamic", motors, SysIdRoutine.Direction.kForward))
        .andThen(new WaitCommand(5))
        .andThen(RobotContainer.shooter.shooterSysId("dynamic", motors, SysIdRoutine.Direction.kReverse));
    }

  /**
   * @param angle     the target angle in field space
   * @param lockDrive should translational motion be locked
   * @return
   */
  public static Command goToAngleFieldRelative(Rotation2d angle, boolean lockDrive) {
    return alignToAngleFieldRelative(
        () -> {
          return Drivetrain.wrapRotation(angle);
        },
        lockDrive);
  }

  public static Command moveToObjectSideways(Limelight camera)
    {
        // final double forwardVelocityClamped = Math555.clamp(forwardVelocity, 0, DriveConstants.MAX_SPEED_MPS);
        Debouncer hasEnded = new Debouncer(0.1, DebounceType.kRising);

        final double DEADBAND = 2;
        final double SPEED_MUL = -DriveConstants.MAX_SPEED * 0.4;

        Command moveSideways = ifHasTarget(
            Commands.runOnce(() -> hasEnded.calculate(false))
                .andThen(Commands.run(() -> RobotContainer.drivetrain.getSwerveDrive().setChassisSpeeds(new ChassisSpeeds(0, -Math555.atLeast(SPEED_MUL * camera.getTXRaw() / 27.0, 0.11), 0)))
                    .until(() -> hasEnded.calculate(Math.abs(camera.getTXRaw()) < DriveConstants.ANGLE_DEADBAND)))
        , camera);



        return moveSideways;
    }

  public static Command alignToAmp() {
    double angle = 90;
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      angle = -90;
    }
    return Commands.sequence(
      // alignToLimelightTarget(RobotContainer.shooterLimelight, DetectionType.APRIL_TAG),
      moveToObjectSideways(RobotContainer.shooterLimelight)
    );
  }

  /**
   * @param angle     the target angle in robot space
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
   * @return a command that will align the robot to the target from the current
   *         limelight Will be
   *         canceled if the limelight loses its target
   */
  public static Command alignToLimelightTarget(Limelight camera, DetectionType targetType) {

    // Rotation2d targetAngle = Rotation2d.fromDegrees(-camera.getObjectTX());
    return Commands.sequence(
        waitForPipe(camera, targetType),
        ifHasTarget(
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
                  camera.setDefaultPipeline();
                })); 
  }

  public static Command alignToLimelightTargetExtreme(Limelight camera) {
    return Commands.sequence(
      waitForPipe(camera, DetectionType.APRIL_TAG),
      ifHasTarget(
        alignToAngleRobotRelativeContinuous(() -> {return Rotation2d.fromDegrees(camera.getHeadingToPriorityID() - RobotContainer.shooterLimelight.maxIsStupid().getDegrees());}, false), camera
      ).finallyDo(
        () -> {
          RobotContainer.drivetrain.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
          camera.setDefaultPipeline();

        }
      )
    );
  }

  public static Command alignToLimelightTargetPose(Limelight camera) {
    return Commands.sequence(
      waitForPipe(camera, DetectionType.APRIL_TAG),
      ifHasTarget(
        alignToAngleRobotRelativeContinuous(() -> {return Rotation2d.fromDegrees(camera.getAngleToSpeaker());}, false), camera
      ).finallyDo(
        () -> {
          RobotContainer.drivetrain.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
          camera.setDefaultPipeline();

        }
      )
    );
  }

  public static Command alignToLimelightTargetWithDrive(Limelight camera, DetectionType targetType, Translation2d speeds) {

    // Rotation2d targetAngle = Rotation2d.fromDegrees(-camera.getObjectTX());
    return Commands.sequence(
        waitForPipe(camera, targetType),
        ifHasTarget(
            alignToAngleRobotRelativeContinuousWithDrive(
                () -> {
                  Rotation2d targetAngle = Rotation2d.fromDegrees(-camera.getObjectTX());
                  return targetAngle;
                },
                speeds),
            camera)
            .finallyDo(
                () -> {
                  RobotContainer.drivetrain.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
                  camera.setDefaultPipeline();
                })); 
  }

  //ONLY USE IN AUTO JANK AF
  public static Command alignToLimelightTargetWithStop(Limelight camera, DetectionType targetType) {

    // Rotation2d targetAngle = Rotation2d.fromDegrees(-camera.getObjectTX());
    
    return Commands.sequence(
        waitForPipe(camera, targetType),
        log("Got pipe in alignToLimelightTargetWithStop"),
        ifHasTarget(
            alignToAngleRobotRelativeContinuous(
                () -> {
                  Rotation2d targetAngle = Rotation2d.fromDegrees(-camera.getObjectTX());
                  return targetAngle;
                },
                false),
            camera)
            .until(camera::isAligned)
            .finallyDo(
                () -> {
                  RobotContainer.drivetrain.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
                  camera.setDefaultPipeline();
                })); 
  }

  public static Command scoreMode() {
    return Commands.parallel(
        alignToAngleRobotRelativeContinuous(() -> Rotation2d.fromDegrees(RobotContainer.shooterLimelight.getHeadingToPriorityID()), false),
        //alignToLimelightTargetExtreme(RobotContainer.shooterLimelight),
        // alignToLimelightTarget(RobotContainer.shooterLimelight, DetectionType.APRIL_TAG),
        setSprocketAngle(RobotContainer.shooterLimelight::bestFit),
        Commands.runOnce(() -> {
          RobotContainer.isDriverMode = true;
          RobotContainer.shooter.shootVelocity(ShooterConstants.SPEAKER_EJECT_SPEED, ShooterConstants.SPEAKER_EJECT_SPEED);
          // RobotContainer.shooter.transportStart(ShooterConstants.TRANSPORT_SPEED);
        })).finallyDo(() -> {
          RobotContainer.shooter.stop();
          RobotContainer.isDriverMode = false;
        });
  }

   public static Command scoreModePose() {
    return Commands.parallel(
        alignToLimelightTargetPose(RobotContainer.shooterLimelight),
        setSprocketAngle(() -> {
          return RobotContainer.shooterLimelight.bestFitFromDistance(RobotContainer.shooterLimelight.getPoseDistanceToSpeaker());
        }),
        Commands.runOnce(() -> {
          RobotContainer.isDriverMode = true;
          RobotContainer.shooter.shootVelocity(ShooterConstants.SPEAKER_EJECT_SPEED, ShooterConstants.SPEAKER_EJECT_SPEED);
          // RobotContainer.shooter.transportStart(ShooterConstants.TRANSPORT_SPEED);
        })).finallyDo(() -> {
          RobotContainer.shooter.stop();
          RobotContainer.isDriverMode = false;
        });
  }
  

  public static Command angleToSpeaker() {
    return Commands555.goToAngleFieldRelative(RobotContainer.drivetrain.getSpeakerAngle(), false);
  }

  // public static Command alignToAmpAndShoot() {
  
  //   Pose2d drivePose = RobotContainer.drivetrain.getSwerveDrive().getPose();

  //   Translation2d targetTranslation = drivePose.getTranslation();
  //   targetTranslation = targetTranslation.plus(RobotContainer.shooterLimelight.getTargetPoseRobotSpace().getTranslation()).minus(new Translation2d(DriveConstants.BUMPER_WIDTH + DriveConstants.DRIVE_BASE_RADIUS, 0));

  //   Rotation2d rot = Drivetrain.flipAngle(Rotation2d.fromDegrees(270));

  //   // targetTranslation.plus(AutoConstants.TRANSLATION_FROM_AMP);

  //   List<Translation2d> points = PathPlannerPath.bezierFromPoses(
  //     drivePose,
  //     new Pose2d(targetTranslation, rot)
  //   );

  //   PathPlannerPath path = new PathPlannerPath(points, AutoConstants.PATH_CONSTRAINTS, new GoalEndState(0, rot));

  //   return Commands.sequence(
  //     Commands.parallel(
  //       AutoBuilder.followPath(path),
  //       setSprocketAngle(ArmConstants.AMP_SCORE_ANGLE),
  //       spinUpShooter(ShooterConstants.AMP_EJECT_SPEED_TOP, ShooterConstants.AMP_EJECT_SPEED_BOTTOM)
  //     ),
  //     shoot(ShooterConstants.AMP_EJECT_SPEED_TOP, ShooterConstants.AMP_EJECT_SPEED_BOTTOM, ShooterConstants.TRANSPORT_SPEED)
  //   );
  // }

  public static Command spinUpShooter(double topSpeed, double bottomSpeed) {
    return Commands.runOnce(() -> {
      RobotContainer.shooter.shootVelocity(topSpeed, bottomSpeed);
    });
  }

  public static Command sysIdDrive() {
    SysIdRoutine routine = RobotContainer.drivetrain.getSysIdDrive();
    return Commands.sequence(
      routine.dynamic(Direction.kForward),
      waitForTime(5),
      routine.dynamic(Direction.kReverse),
      waitForTime(5),
      routine.quasistatic(Direction.kForward),
      waitForTime(5),
      routine.quasistatic(Direction.kReverse),
      waitForTime(5)
    );
  }

  // public static Command addVisionMeasurement() {
  //   return Commands.runOnce(() -> {
  //     LimelightHelpers.PoseEstimate targetPose = RobotContainer.shooterLimelight.getAdjustedPose();
      
  //     RobotContainer.drivetrain.addVisionMeasurement(
  //       targetPose.pose,
  //       targetPose.timestampSeconds,
  //       RobotContainer.shooterLimelight.getVisionStdDevs(targetPose)
  //     );
  //   });
  // }
 
  // Used during auto for scoring speaker(usually from one of the note locations)
  public static Command scoreModeAuto() {
    Command alignAndAngle = alignToAngleRobotRelative(() -> Rotation2d.fromDegrees(RobotContainer.shooterLimelight.getHeadingToPriorityID()), false).deadlineWith(setSprocketAngle(RobotContainer.shooterLimelight::bestFit));
    return Commands.sequence(
      alignAndAngle.withTimeout(1.3), 
      setSprocketAngleWithStop(RobotContainer.shooterLimelight::bestFit),
      log("Scoring mode auto suckers!"),
      // Commands.waitUntil(() -> {
      //   return (RobotContainer.shooterLimelight.isAlignedAuto() && RobotContainer.sprocket.isAtAngle()) && RobotContainer.shooter.isAtSpeed();
      // }).withTimeout(0.5), 
      log("Shooting Auto!"),
      Commands555.shoot(ShooterConstants.SPEAKER_EJECT_SPEED, ShooterConstants.SPEAKER_EJECT_SPEED, ShooterConstants.TRANSPORT_SPEED));


  }

 

 

  public static Command runTransportManual() {
    return Commands.run(() -> {
      RobotContainer.shooter.transportStart(ShooterConstants.TRANSPORT_SPEED);
    }).finallyDo(() -> {
      RobotContainer.shooter.stopTransport();
    });
  }

  // public static Command testPipeSwitch(Limelight camera, DetectionType pipe) {
  // Timer timer = new Timer();
  // return Commands.run(() -> {
  // timer.reset();
  // timer.start();
  // camera.setPipelineTo(pipe);
  // // System.out.println(Timer.getFPGATimestamp());
  // }).until(() -> {return camera.getPipelineType() == pipe;}).finallyDo(() -> {
  // System.out.println(timer.get());
  // timer.stop();
  // });
  // }
  /**
   * A command decorator to cancel a command if the limelight loses its target
   *
   * @param cmd   the command to be decorated
   * @param limey the limelight to be targeted
   * @return
   */
  public static Command ifHasTarget(Command cmd, Limelight limey) {
    return cmd.onlyWhile(() -> limey.hasValidTarget());
  }


  /***
   * @param limey
   * @return A command that drives to the currently targeted april tag
   */
  // public static Command driveToAprilTag(Limelight limey) {
  // double[] aprilTagPoseArray =
  // LimelightHelpers.getLimelightNTDoubleArray(limey.getName(),
  // "targetpose_robotspace");
  // Pose2d aprilTagPose = new Pose2d(new Translation2d(aprilTagPoseArray[0],
  // aprilTagPoseArray[1]), new Rotation2d(aprilTagPoseArray[5]));
  // Pose2d botPose =
  // aprilTagPose.relativeTo(DriveConstants.EDGE_OF_DRIVEBASE);
  // this work the way I think it does
  // return driveToFieldRelativePoint(botPose);

  // }

  /***
   * Enables field relative mode
   * 
   * @return a command that enables field relative control
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
  // return Commands.runOnce(RobotContainer.sprocket::goUp,
  // RobotContainer.sprocket).withName("sprocket up");
  // }

  // public static Command goDown() {
  // return Commands.runOnce(RobotContainer.sprocket::goDown,
  // RobotContainer.sprocket).withName("sprocket down");
  // }

  public static Command stopSprocket() {
    return Commands.runOnce(RobotContainer.sprocket::stop, RobotContainer.sprocket)
        .withName("sprocket stop");
  }

  /**
   * Angle in degrees
   *
   * @param angle target angle
   * @return Command that sets the target sprocket position to the given angle
   */
  public static Command setSprocketAngle(double angle) {
    System.out.println(angle);
    return Commands.runOnce(
        () -> RobotContainer.sprocket.setPosition(Rotation2d.fromDegrees(angle)), RobotContainer.sprocket);
  }

  public static Command setSprocketAngle(DoubleSupplier angle) {
    // System.out.println(angle);
    return Commands.run(
        () -> RobotContainer.sprocket.setPosition(Rotation2d.fromDegrees(angle.getAsDouble())),
        RobotContainer.sprocket);
  }

  public static Command setSprocketAngleWithStop(DoubleSupplier angle) {
    // System.out.println(angle);
    return Commands.run(
        () -> RobotContainer.sprocket.setPosition(Rotation2d.fromDegrees(angle.getAsDouble())),
        RobotContainer.sprocket).until(RobotContainer.sprocket::isAtAngle);
  }

  /*
   * - - - - - - - - - -
   * Shooter Commands
   * - - - - - - - - - -
   */
  /**
   * 
   * @param topShootSpeed    Top Motor (RPM)
   * @param bottomShootSpeed Bottom Motor (RPM)
   * @param transportSpeed   Transport Motor (-1 / 1)
   * @return Command that waits for shooter to reach setpoint RPM, then starts
   *         transport with given speed
   */
  public static Command shoot(double topShootSpeed, double bottomShootSpeed, double transportSpeed) {

    return Commands.sequence(
      log("Starting shoot command!"),
        Commands.runOnce(() -> {
          RobotContainer.shooter.shootVelocity(topShootSpeed, bottomShootSpeed);
        }, RobotContainer.shooter),
        Commands555.waitUntil(RobotContainer.shooter::isAtSpeed),
        log("In shoot command, at speed!"),
        Commands555.transport(transportSpeed),
        log("Transported"),
        Commands555.waitForTime(0.3))
        .finallyDo(() -> {
          RobotContainer.shooter.stopTransport();
          if (!DriverStation.isAutonomous()) {
            System.out.println("Stopping shooter!");
            RobotContainer.shooter.stopShooter();
          }
        });
  }

  public static Command lowerRobot() {
    return Commands.parallel(
      setSprocketAngle(ArmConstants.SPROCKET_STAGE_ANGLE),
      climbersDown()
    );
  }

  public static Command leftClimberUp() {
    return Commands.run(() -> {
      RobotContainer.climbers.leftUp();
    }).finallyDo(RobotContainer.climbers::leftStop);
  }

  public static Command leftClimberDown() {
    return Commands.run(() -> {
      RobotContainer.climbers.leftDown();
    }).finallyDo(RobotContainer.climbers::leftStop);
  }

  public static Command rightClimberUp() {
    return Commands.run(() -> {
      RobotContainer.climbers.rightUp();
    }).finallyDo(RobotContainer.climbers::rightStop);
  }

  public static Command rightClimberDown() {
    return Commands.run(() -> {
      RobotContainer.climbers.rightDown();
    }).finallyDo(RobotContainer.climbers::rightStop);
  }

  public static Command shoot(double speed, double transportSpeed) {
    return shoot(speed, speed, transportSpeed);
  }

  public static Command log(String msg) {
    return Commands.runOnce(() -> {
      System.out.println(msg);
    });
  }

  public static Command setAutoPose(String autoString) {
    return Commands.runOnce(() -> {
      Pose2d startPose = new Pose2d();
      // if (RobotContainer.shooterLimelight.hasValidTarget() &&
      // RobotContainer.shooterLimelight.getPipelineType() == DetectionType.APRIL_TAG)
      // {
      // startPose = RobotContainer.shooterLimelight.getBotPose();
      // RobotContainer.field.setRobotPose(startPose);
      // } else if (RobotContainer.intakeLimelight.hasValidTarget() &&
      // RobotContainer.intakeLimelight.getPipelineType() == DetectionType.APRIL_TAG)
      // {
      // startPose = RobotContainer.intakeLimelight.getBotPose();
      // }
      // else {
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        switch (autoString.charAt(0)) {
          case '1':
            startPose = AutoConstants.POSE_1_RED;
            break;
          case '2':
            startPose = AutoConstants.POSE_2_RED;
            break;
          case '3':
            startPose = AutoConstants.POSE_3_RED;
            break;
          case '4':
            startPose = AutoConstants.POSE_4_RED;
            break;
        }
      } else {
        switch (autoString.charAt(0)) {
          case '1':
            startPose = AutoConstants.POSE_1;
            break;
          case '2':
            startPose = AutoConstants.POSE_2;
            break;
          case '3':
            startPose = AutoConstants.POSE_3;
            break;
          case '4':
            startPose = AutoConstants.POSE_4;
            break;
        }
      }
    // }
      RobotContainer.drivetrain.getSwerveDrive().resetOdometry(startPose);
    });
  }

  public static Command stopTransport() {
    return Commands.runOnce(() -> {
      RobotContainer.shooter.stopTransport();
    }, RobotContainer.shooter);
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

  // public static Command shootVelocity(double velocity) {
  //   return Commands.runOnce(
  //       () -> {
  //         RobotContainer.shooter.shootVelocity(velocity);
  //       });
  // }

  public static Command waitUntil(BooleanSupplier condition) {
    return new Command() {
      @Override
      public boolean isFinished() {
        return condition.getAsBoolean();
      }
    };
  }

  public static Command waitForPipe(Limelight camera, DetectionType pipeline) {
    return Commands.run(() -> {
    }).beforeStarting(() -> {
      camera.setPipelineTo(pipeline);
    }).until(() -> {
      return camera.getPipelineType() == pipeline; 
    });
  }

  /**
   * 
   * Used during auton for intaking notes using vision.
   * 
   * @return Command that aligns using intake limelight, sets sprocket to intake
   *         angle, and drives forward until either note is intaked or timeout is
   *         reached
   */
  public static Command autonomousIntake() {
    return Commands.sequence(
        Commands555.alignToLimelightTarget(RobotContainer.intakeLimelight, DetectionType.NOTE),
        new WaitUntilCommand(RobotContainer.sprocket::isAtAngle),
        Commands.parallel(
            Commands555.intake(),
            driveFromSpeeds(new ChassisSpeeds(AutoConstants.INTAKING_MOVE_SPEED, 0, 0)))
            .until(() -> RobotContainer.shooter.isNoteInTransport())
            .withTimeout(AutoConstants.INTAKING_TIMEOUT)
            .finallyDo(() -> {
              RobotContainer.shooter.stopTransport();
              RobotContainer.intake.stop();
              RobotContainer.drivetrain.getSwerveDrive().drive(new ChassisSpeeds());
            }));
  }

  public static Command driveFromSpeeds(ChassisSpeeds speeds) {
    return Commands.runOnce(() -> RobotContainer.drivetrain.getSwerveDrive().drive(speeds));
  }

  public static Command driveFromSpeedsSet(ChassisSpeeds speeds) {
    // return Commands.run(() -> RobotContainer.drivetrain.getSwerveDrive().drive(speeds));
    return Commands.run(() -> {
      RobotContainer.drivetrain.drive(new Translation2d(2, 0), 0);
    });
  }

  public static Command waitForTime(double seconds) {
    return Commands.run(() -> {
    }).withTimeout(seconds);
  }

  public static Command transport(double transportSpeed) {
    return Commands.runOnce(
        () -> {
          RobotContainer.shooter.transportStart(transportSpeed);
        });
  }
  // public static Command translateToAmpCommand() {
    
  // }
  public static Command scoreAmp() {
    return Commands.sequence(
        setSprocketAngle(ArmConstants.AMP_SCORE_ANGLE),
        waitUntil(RobotContainer.sprocket::isAtAngle),
        shoot(ShooterConstants.AMP_EJECT_SPEED_TOP, ShooterConstants.AMP_EJECT_SPEED_BOTTOM, ShooterConstants.TRANSPORT_SPEED));
    // setSprocketAngle(ArmConstants.INTAKE_ANGLE));

  }
  
  public static Command scoreAmpAuto() {
    return Commands.sequence(
      scoreAmp(),
      
      Commands.runOnce(() -> {
        RobotContainer.shooter.shootVelocity(ShooterConstants.SPEAKER_EJECT_SPEED, ShooterConstants.SPEAKER_EJECT_SPEED);
      })
    );
  }

  
  public static Command scoreSubwoofer() {
    return Commands.sequence(
        Commands.runOnce(() -> System.out.println("Scoring subwoofer!")),
        setSprocketAngle(ArmConstants.SPEAKER_SCORE_ANGLE),
        waitUntil(() -> {
          return RobotContainer.sprocket.isAtAngle();
        }),
        shoot(ShooterConstants.SPEAKER_EJECT_SPEED, ShooterConstants.SPEAKER_EJECT_SPEED,
            ShooterConstants.TRANSPORT_SPEED),
        Commands.runOnce(() -> System.out.println("Done Shooting subwoofer!")),
        setSprocketAngle(ArmConstants.INTAKE_ANGLE)
        );
  }

  // public static Command scoreSubwooferAtAngle() {
  //   return Commands.sequence(
  //       Commands.runOnce(() -> System.out.println("Shooting!")),
  //       setSprocketAngle(ArmConstants.SPEAKER_SCORE_ANGLE),
  //       waitUntil(() -> {
  //         return RobotContainer.sprocket.isAtAngle();
  //       }),
  //       shoot(ShooterConstants.SPEAKER_EJECT_SPEED, ShooterConstants.SPEAKER_EJECT_SPEED,
  //           ShooterConstants.TRANSPORT_SPEED),
  //       Commands.runOnce(() -> System.out.println("Done Shooting!")),
  //       setSprocketAngle(ArmConstants.INTAKE_ANGLE));
  // }

  

 
  public static Command receiveHumanPlayerNote() {
    return Commands.sequence(
        alignToLimelightTarget(RobotContainer.shooterLimelight, DetectionType.APRIL_TAG),
        setSprocketAngle(ArmConstants.SPEAKER_SCORE_ANGLE),
        reverseShooter(),
        setSprocketAngle(ArmConstants.ENCODER_MIN_ANGLE));
  }

  // public static Command signalAmp() {
  // return Commands.runOnce(
  // () -> {
  // RobotContainer.led.add(new FlashAnimation(Color.kOrange));
  // });
  // }

  // public static Command signalCoop() {
  // return Commands.runOnce(
  // () -> {
  // RobotContainer.led.add(new FlashAnimation(Color.kBlue));
  // });
  // }

  // // LED bits
  // public static Command celebrate() {
  // return Commands.runOnce(
  // () -> {
  // RobotContainer.led.add(new CelebrationAnimation());
  // });
  // }

  // public static Command ampItUp() {
  // return Commands.runOnce(
  // () -> {
  // RobotContainer.led.add(new FlashAnimation(Color.kYellow));
  // });
  // }

  // public static Command cooperatition() {
  // return Commands.runOnce(
  // () -> {
  // RobotContainer.led.add(new FlashAnimation(Color.kBlueViolet));
  // });
  // }

  // ***********************CLIMBER COMMANDS*************************//
  public static Command climbersUp() {
    return Commands.run(
        () -> RobotContainer.climbers.up()
    );//.until(() -> RobotContainer.climbers.atTop());
  }

  public static Command climbersDown() {
    return Commands.run(() -> RobotContainer.climbers.down()
    );//.until(() -> RobotContainer.climbers.atBottom());
  }

  // public static Command climbersFullDown() {
  //   return climbersDown().until(() -> RobotContainer.climbers.atBottom && RobotContainer.climbers.getLimits());
  // }

  // public static Command climbersStop() {
  //   return Commands.runOnce(() -> {
  //     RobotContainer.climbers.stop();
  //   });
  // }
}
