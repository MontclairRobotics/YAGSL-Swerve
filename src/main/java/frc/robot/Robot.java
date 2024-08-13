// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  

  @Override
  public void robotInit() {
    // Logger.recordMetadata("ProjectName", "Katherine Tchaikovsky Swift");

    if (isReal()) {
      //Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      //Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
     
      
    } else {
      // setUseTiming(false); // Run as fast as possible
      // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope
      // (or prompt the user)
      // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); //
      // Save outputs to a new log
    }

    // Logger.start();

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    RobotContainer.shooterLimelight.setDefaultPipeline();
    RobotContainer.intakeLimelight.setDefaultPipeline();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.shooter.teleopInit();
    RobotContainer.intake.teleopInit();
    // RobotContainer.shooterLimelight.setDefaultPipeline();
    // RobotContainer.intakeLimelight.setDefaultPipeline();
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  //  System.out.println(RobotContainer.shooterLimelight.getVerticalToPriorityID());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    
    
  }

  @Override
  public void testPeriodic() {
    
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  @Override
  public void autonomousExit() {
    
    // if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
    //   Rotation2d rot = Rotation2d.fromDegrees(180).minus(RobotContainer.drivetrain.getSwerveDrive().getOdometryHeading());
    //   Pose2d pose = new Pose2d(RobotContainer.drivetrain.getSwerveDrive().getPose().getTranslation(), rot);
    //   RobotContainer.drivetrain.getSwerveDrive().resetOdometry(pose);
    // }
  }
}
