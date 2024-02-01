// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sprocket;
import frc.robot.vision.Limelight;

import java.io.File;

import org.littletonrobotics.junction.inputs.LoggedDriverStation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;


public class RobotContainer {

  private static CommandPS5Controller driverController = new CommandPS5Controller(0);
  
  
  public static Drivetrain drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(), "swerve/"));
  
  // Subsystems
  public static Intake intake = new Intake();
  public static Shooter shooter = new Shooter();
  public static Sprocket sprocket = new Sprocket();
  public static Limelight intakeLimelight = new Limelight("intakeLimelight");
  public static Limelight shooterLimelight = new Limelight("shooterLimelight");
  public static Auto auto = new Auto();

  public static final Field2d field = new Field2d();

  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    auto.setupPathPlanner();
    setupAutoTab();
    
    drivetrain.setDefaultCommand(Commands.run(() -> {
    drivetrain.setInputFromController(driverController);


      
    }, drivetrain));
    configureBindings();
  }


  private void configureBindings() {
    driverController.cross().onTrue(new InstantCommand(() -> {

      Translation2d targetPose = new Translation2d(0.33, 0.33);
      Rotation2d currentRotation = drivetrain.getRotation();
      Commands555.driveToRobotRelativePoint(targetPose, currentRotation);

    }));
    driverController.circle().onTrue(Commands.runOnce(() -> {

      Translation2d targetPose = new Translation2d(0.33,0);
      Rotation2d currentRotation = drivetrain.getRotation();
      Commands555.driveToRobotRelativePoint(targetPose, currentRotation.rotateBy(new Rotation2d(90)));

    }));
    driverController.touchpad().onTrue(Commands.runOnce(() -> {
      drivetrain.getSwerveDrive().zeroGyro();
    }));
    driverController.triangle().onTrue(Commands.run(() -> {
      System.out.println("BUTTON PRESSED");
      Translation2d targetPose = new Translation2d(0.33,0);
      Rotation2d currentRotation = drivetrain.getRotation();
      Commands555.driveToRobotRelativePoint(targetPose, currentRotation);
     
    }, drivetrain));
  }

  public void setupAutoTab() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    //TODO make it the same string that was entered last time? I think i can mark nt key as persistent
    autoTab.add("Enter Command", "").withSize(3,1).withPosition(0,0);
    autoTab.add(field).withSize(6,4).withPosition(3,0);
    autoTab.addString("Feedback", () -> auto.getFeedback()).withSize(3,1).withPosition(0, 1);
    
    autoTab.add("Ignore Safety", false).withWidget(BuiltInWidgets.kToggleSwitch).withSize(2, 1).withPosition(0,2);


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TODO: Actual auto command.
    return Commands.run(() -> {
      return;
    }); 

    
  }
}
