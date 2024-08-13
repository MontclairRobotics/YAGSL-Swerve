// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import animation2.AllianceAnimation;
import animation2.WipeTransition;
import animation2.api.Animation;
import animation2.api.ConditionalAnimation;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sprocket;
import frc.robot.util.ControllerTools;
import frc.robot.util.ControllerTools.DPad;
import frc.robot.util.Tunable;
import frc.robot.vision.DetectionType;
import frc.robot.vision.Limelight;

public class RobotContainer {

  public static CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
  // public static CommandPS5Controller debugController = new CommandPS5Controller(2);

  public static XboxController testController = new XboxController(2);
  public static Drivetrain drivetrain =
      new Drivetrain(new File(Filesystem.getDeployDirectory(), "swerve/"));

  // Subsystems
  public static Intake intake = new Intake();
  public static Shooter shooter = new Shooter();
  public static Sprocket sprocket = new Sprocket();
  public static Limelight intakeLimelight = new Limelight("limelight-intake", DetectionType.NOTE);
  public static Limelight shooterLimelight = new Limelight("limelight-shooter", DetectionType.APRIL_TAG);
  public static Auto auto = new Auto();

  public static Climbers climbers = new Climbers();
  

  public static final Field2d field = new Field2d();

  

  public static boolean isDriverMode = false;

  public RobotContainer() {
    auto.setup();
    
    

    drivetrain.setDefaultCommand(
        Commands.run(
            () -> {
              drivetrain.setInputFromController(driverController);
            },
            drivetrain));

    sprocket.setDefaultCommand(
        Commands.run(
            () -> {
              if (!operatorController.getHID().getL1Button()) {
              //if (!operatorController.L1().getAsBoolean()) {
                sprocket.setInputFromJoystick(operatorController);
              }
            },
            sprocket));

    climbers.setDefaultCommand(
      Commands.run(
        () -> {
          if (!operatorController.getHID().getL1Button()) {
          //if (!operatorController.L1().getAsBoolean()) {
            climbers.setInputFromController(operatorController);
          } else {
            climbers.setInputFromSticks(operatorController);
          }
        },
        climbers
      )
    );

    shooterLimelight.setPipelineTo(DetectionType.APRIL_TAG);
    intakeLimelight.setPipelineTo(DetectionType.NOTE);

    configureDriverBindings();
    configureOperatorBindings();
    configureDebugBindings();
  }

  private void configureDebugBindings() {
    // ControllerTools.getDPad(DPad.UP, debugController).whileTrue(Commands555.leftClimberUp());
    // ControllerTools.getDPad(DPad.DOWN, debugController).whileTrue(Commands555.leftClimberDown());

    // ControllerTools.getDPad(DPad.LEFT, debugController).whileTrue(Commands555.rightClimberUp());
    // ControllerTools.getDPad(DPad.RIGHT, debugController).whileTrue(Commands555.rightClimberDown());

    // debugController.cross().whileTrue(Commands555.driveOneMeter());
    // debugController.circle().whileTrue(Commands555.scoreModeAuto());
  }

  private void configureDriverBindings() {

    // ************* DRIVER CONTROLLER BINDINGS **************** //
    driverController
        .L2()
        .onTrue(Commands555.disableFieldRelative())
        .onFalse(Commands555.enableFieldRelative());
    

    driverController
        .triangle()
        .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(0), false));
    driverController
        .circle()
        .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(-90), false));
    driverController
        .cross()
        .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(180), false));
    driverController
        .square()
        .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(90), false));

    driverController.R2().and(() -> shooterLimelight.hasValidTarget()).whileTrue(Commands555.scoreMode());
    driverController.L1().whileTrue(Commands555.alignToLimelightTarget(shooterLimelight, DetectionType.APRIL_TAG));
    driverController.R1().whileTrue(Commands555.alignToLimelightTarget(intakeLimelight, DetectionType.NOTE));

    driverController
        .touchpad()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drivetrain.getSwerveDrive().zeroGyro();
                    })
                .ignoringDisable(true));
    driverController.PS().onTrue(Commands555.lockDrive());

    // ControllerTools.getDPad(DPad.LEFT, driverController).onTrue(Commands555.loadNoteAuto());

    ControllerTools.getDPad(DPad.LEFT, driverController).onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(60), false));
    ControllerTools.getDPad(DPad.RIGHT, driverController).onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(300), false));
    ControllerTools.getDPad(DPad.UP, driverController).onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(180), false));
    ControllerTools.getDPad(DPad.DOWN, driverController).whileTrue(Commands555.loadNoteAuto().withTimeout(1.5));
                    
  }
  private void configureOperatorBindings() {
  
    // ControllerTools.getDPad(DPad.UP, operatorController).onTrue(Commands555.climbersUp());
    // ControllerTools.getDPad(DPad.DOWN, operatorController).onTrue(Commands555.climbersDown());

    operatorController.R2().whileTrue(Commands555.unloadNote());
    // operatorController.R2().onTrue(Commands555.testPipeSwitch(intakeLimelight, DetectionType.APRIL_TAG));
    operatorController.L2().whileTrue(Commands555.loadNote());

    // ControllerTools.getDPad(DPad.UP, debugController).whileTrue(Commands555.leftClimberUp());
    // ControllerTools.getDPad(DPad.DOWN, debugController).whileTrue(Commands555.leftClimberDown());

    // ControllerTools.getDPad(DPad.LEFT, debugController).whileTrue(Commands555.rightClimberUp());
    // ControllerTools.getDPad(DPad.RIGHT, debugController).whileTrue(Commands555.rightClimberDown());

    
      
    // operatorController.square().onTrue(Commands555.setSprocketAngle(shooterLimelight.bestFit()));
    // operatorController.cross().onTrue(Commands.runOnce(() -> {     
    //   System.out.println(RobotContainer.sprocket.angleSetpoint.get()); 
    //   RobotContainer.sprocket.setPosition(Rotation2d.fromDegrees(RobotContainer.sprocket.angleSetpoint.get()));
    // }));

    operatorController.cross().whileTrue(Commands555.ferryNote(42));
    operatorController.triangle().onTrue(Commands555.scoreAmp());
    operatorController.square().onTrue(Commands555.ferryNote(32));
    // operatorController.square().onTrue(Commands555.spinUpShooter(4000, 4000)).onFalse(Commands555.stopShooter());
    operatorController.R1().whileTrue(Commands555.loadNoteSource());
    
    

    // operatorController.circle().and(() -> !isDriverMode).onTrue(Commands555.shoot(ShooterConstants.SPEAKER_EJECT_SPEED, ShooterConstants.SPEAKER_EJECT_SPEED, ShooterConstants.TRANSPORT_SPEED, 1));
    operatorController.circle().and(() -> !isDriverMode).whileTrue(Commands555.scoreSubwoofer());
    operatorController.circle().and(() -> isDriverMode).whileTrue(Commands555.runTransportManual());
    
    // operatorController.R1().onTrue(Commands.runOnce(RobotContainer.shooter::toggleShooter)); //TODO 
   
    // ControllerTools.getDPad(DPad.LEFT, operatorController).onTrue(Commands.runOnce(drivetrain::playMusic).ignoringDisable(true));


    // operatorController.L1().onTrue(Commands555.celebrate());
    // operatorController.touchpad().onTrue(Commands555.ampItUp());
    // operatorController.PS().onTrue(Commands555.cooperatition());

  }

  public static Animation getTeleopDefaultAnim() {
    return new AllianceAnimation();
  }

  public static Animation getDisabledAnimation() {
    return Constants.LEDConstants.DEMO_REEL;
  }

  public void setupDriverTab() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addBoolean(
        "Good to shoot!",
        () -> {
          return sprocket.isAtAngle() && Math.abs(shooterLimelight.getObjectTX()) < VisionConstants.ALIGN_CENTER_OFFSET;
        });

    driverTab.addDouble("Time Remaining", () -> { return (int) Timer.getMatchTime();});

    driverTab.addBoolean("Note in Transport", shooter::isNoteInTransport);

    // driverTab.addDouble("Match Time", () -> {
    //   return Timer.getMatchTime();
    // });
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // PathPlannerPath path = PathPlannerPath.fromPathFile("A-5");
    
    // return Commands.sequence(
    //   Commands.runOnce(() -> {
    //     RobotContainer.drivetrain.getSwerveDrive().resetOdometry(path.getPreviewStartingHolonomicPose());
    //   }),
    //   AutoBuilder.followPath(path)
    // );
    return auto.getAutoCommand();
  }
}
