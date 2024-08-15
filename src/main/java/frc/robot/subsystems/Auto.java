package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.PathPlannerLogging;

import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;


import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.networktables.NetworkTableInstance;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands555;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

import frc.robot.util.Array555;

// import static frc.robot.Constants.ArmConstants.INTAKE_SCORE_ANGLE;


import frc.robot.Constants.DriveConstants;


import java.util.Optional;


import org.littletonrobotics.junction.Logger;



public class Auto extends SubsystemBase {

  public SendableChooser<Command> autoChooser;

  public Auto() {
    
    
    
  }

  public Command getAutoCommand() {
    return autoChooser.getSelected();
    
  }

  private void setupAutoSelector() {
    autoChooser = AutoBuilder.buildAutoChooser(); 
    autoChooser.setDefaultOption("Preloaded Only", Commands555.scoreSubwoofer());
    Shuffleboard.getTab("Auto").add("Auto Chooser", autoChooser);
    

  }


  public void setup() {
    setupPathPlanner();
    registerNamedCommands();
    setupAutoSelector();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("AlignAndShoot", Commands555.alignAndShootAuto());
    NamedCommands.registerCommand("LoadNote", Commands555.loadNote());
    NamedCommands.registerCommand("ScoreSubwoofer", Commands555.scoreSubwoofer());
    NamedCommands.registerCommand("SpinupShooter", Commands555.spinUpShooter(ShooterConstants.SPEAKER_EJECT_SPEED, ShooterConstants.SPEAKER_EJECT_SPEED));
  }

  private void setupPathPlanner() {
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


    
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Auto/ActivePath", activePath.toArray(new Pose2d[activePath.size()]));
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Auto/TargetPose", targetPose);
        });
  }


}
  

