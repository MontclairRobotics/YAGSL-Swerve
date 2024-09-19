// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import animation2.AllianceAnimation;

import animation2.api.Animation;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sprocket;
import frc.robot.util.ControllerTools;
import frc.robot.util.ControllerTools.DPad;
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

    auto.setupPathPlanner();
    auto.setupAutoTab();
    setupDriverTab();
    
    Shuffleboard.getTab("Debug").addDouble("Distance", () -> {return shooterLimelight.getDistanceToSpeaker();});

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

  }

  private void configureDriverBindings() {

    // ************* DRIVER CONTROLLER BINDINGS **************** //
    driverController
        .L2()
        .onTrue(Commands555.disableFieldRelative())
        .onFalse(Commands555.enableFieldRelative());
    
    
    
    

    // driverController
    //     .triangle()
    //     .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(0), false));
    // driverController
    //     .circle()
    //     .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(-90), false));
    // driverController
    //     .cross()
    //     .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(180), false));
    // driverController
    //     .square()
    //     .onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(90), false));


    BooleanSupplier isOnBlueAlliance = () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue;

    driverController
      .triangle()
      .onTrue(new ConditionalCommand(
        Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(0), false), 
        Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(180), false), 
        isOnBlueAlliance));
      
    driverController
      .circle()
      .onTrue(new ConditionalCommand(
        Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(-90), false), 
        Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(90), false), 
        isOnBlueAlliance));
    
    driverController
      .cross()
      .onTrue(new ConditionalCommand(
        Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(180), false), 
        Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(0), false), 
        isOnBlueAlliance));

    driverController
        .square()
        .onTrue(new ConditionalCommand(
          Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(90), false), 
          Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(-90), false),
          isOnBlueAlliance));


    driverController.R2().and(() -> shooterLimelight.hasValidTarget()).whileTrue(Commands555.scoreMode());
    driverController.L1().whileTrue(Commands555.alignToLimelightTarget(shooterLimelight, DetectionType.APRIL_TAG));
    driverController.R1().whileTrue(Commands555.alignToLimelightTarget(intakeLimelight, DetectionType.NOTE));

    driverController
        .touchpad()
        .onTrue(Commands555.zeroGyro()
                .ignoringDisable(true));
    driverController.PS().onTrue(Commands555.lockDrive());

    
    
    if (isOnBlueAlliance.getAsBoolean()) {
      ControllerTools.getDPad(DPad.LEFT, driverController).onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(60), false));
      ControllerTools.getDPad(DPad.RIGHT, driverController).onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(300), false));
      ControllerTools.getDPad(DPad.UP, driverController).onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(180), false));
    } else {
      ControllerTools.getDPad(DPad.LEFT, driverController).onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(-120), false));
      ControllerTools.getDPad(DPad.RIGHT, driverController).onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(120), false));
      ControllerTools.getDPad(DPad.UP, driverController).onTrue(Commands555.goToAngleFieldRelative(Rotation2d.fromDegrees(0), false));
    }

    ControllerTools.getDPad(DPad.DOWN, driverController).whileTrue(Commands555.alignToAmp());
    
                    
  }
  private void configureOperatorBindings() {
  
   

    operatorController.R2().whileTrue(Commands555.unloadNote());
    operatorController.L2().whileTrue(Commands555.loadNote());



    operatorController.cross().whileTrue(Commands555.ferryNote(42));
    operatorController.triangle().onTrue(Commands555.scoreAmp());
    operatorController.square().onTrue(Commands555.ferryNote(32));
    operatorController.R1().whileTrue(Commands555.loadNoteSource());
    
    

   
    operatorController.circle().and(() -> !isDriverMode).whileTrue(Commands555.scoreSubwoofer());
    operatorController.circle().and(() -> isDriverMode).whileTrue(Commands555.runTransportManual());
    


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

 
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return Commands555.setAutoPose("2");
    return auto.getAutoCommand();
  }
}
