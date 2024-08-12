package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;



import edu.wpi.first.math.geometry.Translation2d;


import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import frc.robot.Constants.DriveConstants;

import java.util.Optional;


public class Auto extends SubsystemBase {

  private final SendableChooser<Command> autoChooser;
  
  public Auto() {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
    
  }

  public Command getAutoCommand() {
    return autoChooser.getSelected();
  }

  public void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
        RobotContainer.drivetrain.getSwerveDrive()::getPose, // Robot pose supplier
        RobotContainer.drivetrain.getSwerveDrive()
            ::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        RobotContainer.drivetrain.getSwerveDrive()
            ::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (ChassisSpeeds x) -> {
          RobotContainer.drivetrain.getSwerveDrive().drive(new ChassisSpeeds(x.vxMetersPerSecond, x.vyMetersPerSecond, x.omegaRadiansPerSecond), DriveConstants.IS_OPEN_LOOP, new Translation2d());
        }, // Method that will drive the robot given ROBOT RELATIVE, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        Constants.AutoConstants.PATH_FOLLOWER_CONFIG,
        () -> {
          Optional<Alliance> alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        RobotContainer.drivetrain);
  }

}
  

