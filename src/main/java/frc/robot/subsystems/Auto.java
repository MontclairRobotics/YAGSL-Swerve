package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.kinematics.ChassisSpeeds;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
// import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
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

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;


// ^(?=[^A-H1-4]*[A-H1-4])(?=[^0-9]*[0-9])(?!.*[A-Za-z]{2})(?!.*[0-9]{2})[A-H1-4]+$
public class Auto extends SubsystemBase {

  private ArrayList<PathPlannerTrajectory> trajectories = new ArrayList<PathPlannerTrajectory>();
  private String feedbackValue = "Enter a command!";
  private Command autoCommand = Commands.runOnce(() -> {});
  private Alliance alliance = Alliance.Blue;

  GenericEntry autoStringEntry;
  GenericEntry safetyEntry;

  public Auto() {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    NetworkTable table = nt.getTable("Shuffleboard").getSubTable("Auto");

    autoStringEntry = table.getTopic("Enter Command").getGenericEntry();
    safetyEntry = table.getTopic("Ignore Safety").getGenericEntry();
    // Shuffleboard.getTab("Auto").addBoolean("Is Valid", () -> {
    //   return isValidPathSequence(autoStringEntry.getString(""));
    // });
    // Shuffleboard.getTab("Auto").addBoolean("Is Safe", () -> {
    //   return isStayingInLane(autoStringEntry.getString(""));
    // });
    
  }


  public Command getAutoCommand() {
    return autoCommand;
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

  // 1A2
  public boolean isValidPathSequence(String autoString) {
    // Checks if the first point in the string is a starting point
    if (autoString.length() == 0) return true;
    if (Array555.indexOf(Constants.AutoConstants.STARTING_POINTS, autoString.charAt(0)) == -1) {
      setFeedback("That's not a real starting spot.");
      return false;
    }
    // Loops through the string.
    for (int i = 0; i < autoString.length() - 1; i++) {
      char char1 = autoString.charAt(i);
      char char2 = autoString.charAt(i + 1);
      // Checks if there are two notes in a row (not good)
      if (Array555.indexOf(Constants.AutoConstants.NOTES, char1) != -1
          && Array555.indexOf(Constants.AutoConstants.NOTES, char2) != -1) {
        setFeedback("Don't go from a note to a note.");
        clearAll();
        return false;
      }
      // Checks if there are two scoring spots in a row (not good)
      if (Array555.indexOf(Constants.AutoConstants.SCORING_LOCATIONS, char1) != -1
          && Array555.indexOf(Constants.AutoConstants.SCORING_LOCATIONS, char2) != -1) {
        setFeedback("Don't go between scoring locations.");
        clearAll();
        return false;
      }
      // Checks for a typo (or people messing around and typing nonsense)
      if (Array555.indexOf(Constants.AutoConstants.ALL_POINTS, char2) == -1) {
        setFeedback("You probably made a typo, or you're stupid");
        clearAll();
        return false;
      }
    }
    return true;
    // Trajectory traj = new Trajectory(trajectories.get(1).getStates());
  }

  public void drawPaths() {
    clearField();
    System.out.println("DRAWING");
    for (int i = 0; i < trajectories.size(); i++) {
      PathPlannerTrajectory pathTraj = trajectories.get(i);
      List<State> states = convertStatesToStates(pathTraj.getStates());
      Trajectory displayTrajectory = new Trajectory(states);
      
      
      // if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        
      //   // displayTrajectory = displayTrajectory.relativeTo(new Pose2d(33, 0, Rotation2d.fromDegrees(180)));
      //   displayTrajectory = displayTrajectory.relativeTo(new Pose2d(33,16, Rotation2d.fromDegrees(180)));
      // }

      RobotContainer.field.getObject("traj" + i).setTrajectory(displayTrajectory);
      // Shuffleboard.getTab("Auto").add(RobotContainer.field).withSize(6, 4).withPosition(3, 0);
      // RobotContainer.field.
    }
    // ppState.ge
  }

  public void clearField() {
      for (int i  = 0; i < 100; i++) {
        FieldObject2d obj = RobotContainer.field.getObject("traj" + i);
        // obj.setPose(new Pose2d(-100, -100, Rotation2d.fromDegrees(0)));
        obj.setTrajectory(new Trajectory());
      }
  }

  public void clearAll() {
    trajectories.clear();
    clearField();
  }

  public boolean isStayingInLane(String autoString) {
    char[] lane;
    if (autoString.length() == 0) return true; 
    
    // sets lane to the lane of the first character
    if (autoString.charAt(0) == '1') lane = Constants.AutoConstants.LANE1;
    else if (autoString.charAt(0) == '2') lane = Constants.AutoConstants.LANE2;
    else if (autoString.charAt(0) == '3' || autoString.charAt(0) == '4') lane = Constants.AutoConstants.LANE3;
    else return false;
    
    // loops through the string
    for (int i = 0; i < autoString.length(); i++) {

      char character = autoString.charAt(i);
      // Checks if character is in the lane
      if (Array555.indexOf(lane, character) == -1) {
        setFeedback("STAY IN YOUR LANE!!!");
        clearAll();
        return false;
      }
    }
    return true;
  }

  public void setFeedback(String feedback) {
    feedbackValue = feedback;
  }

  public String getFeedback() {
    return feedbackValue;
  }

  public void setupAutoTab() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
   
    // persistent
    autoTab.add("Enter Command", "").withSize(3, 1).withPosition(0, 0);
    autoTab.add(RobotContainer.field).withSize(6, 4).withPosition(3, 0);
    autoTab.addString("Feedback", () -> feedbackValue).withSize(3,1).withPosition(0,1);
    // autoTab.addString("Feedback", () -> auto.getFeedback()).withSize(3,1).withPosition(0, 1);

    autoTab
        .add("Ignore Safety", false)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .withSize(2, 1)
        .withPosition(0, 2);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable ntTable = inst.getTable("Shuffleboard").getSubTable("Auto");

    ntTable.addListener(
        "Enter Command",
        EnumSet.of(Kind.kValueAll),
        (table, key, event) -> {
         validateAndCreatePaths();
        });
       
    ntTable.addListener(
        "Ignore Safety",
        EnumSet.of(Kind.kValueAll),
        (table, key, event) -> {
         validateAndCreatePaths();
        });    
    
  }

  public List<State> convertStatesToStates(List<PathPlannerTrajectory.State> ppStates) {
    ArrayList<State> wpiStates = new ArrayList<State>();
    for (int i = 0; i < ppStates.size(); i++) {
      // PathPlannerPath
      PathPlannerTrajectory.State currentState = ppStates.get(i);
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        wpiStates.add(new State(
          currentState.timeSeconds,
          currentState.velocityMps,
          currentState.accelerationMpsSq,
          new Pose2d(16.54175-currentState.positionMeters.getX(), currentState.positionMeters.getY(), currentState.heading),
          currentState.curvatureRadPerMeter
      ));
      } else {
        wpiStates.add(new State(
          currentState.timeSeconds,
          currentState.velocityMps,
          currentState.accelerationMpsSq,
          new Pose2d(currentState.positionMeters.getX(), currentState.positionMeters.getY(), currentState.heading),
          currentState.curvatureRadPerMeter
      ));
      }
    }

    return wpiStates;
  }



  public void validateAndCreatePaths() {
    String autoString = autoStringEntry.getString("");
    // previousString = autoString;

    boolean ignoreSafety = safetyEntry.getBoolean(false);


    boolean isSafePath = false;
    if (ignoreSafety) {
      isSafePath = true;
    } else {
      isSafePath = isStayingInLane(autoString); // :)
    }
    
    boolean isValidPath = true;//isValidPathSequence(autoString);
    // setFeedback("boo!");
    if (isValidPath) {
      if (!ignoreSafety && isSafePath) {
        buildPathSequenceOdometry(autoString);
        drawPaths();
      } else if (ignoreSafety) {
        buildPathSequenceOdometry(autoString);
        drawPaths();
      }
    }
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() != alliance) {
        alliance = DriverStation.getAlliance().get();
        validateAndCreatePaths();
      }
  }
    
  }

  public void buildPathSequenceOdometry(String autoString) {

    SequentialCommandGroup finalPath = new SequentialCommandGroup();
    
    trajectories.clear();

    if (autoString.length() == 0) {
      autoCommand = Commands.sequence(Commands555.setAutoPose(autoString), Commands555.scoreSubwoofer());
      return;
    }

    finalPath.addCommands(Commands555.setAutoPose(autoString));
    finalPath.addCommands(Commands.runOnce(() -> {
      RobotContainer.shooter.shootVelocity(ShooterConstants.SPEAKER_EJECT_SPEED, ShooterConstants.SPEAKER_EJECT_SPEED);
    }));
    

    if (autoString.charAt(0) == '4') {
      finalPath.addCommands(Commands555.scoreAmp());
    } else {
      finalPath.addCommands(Commands555.scoreSubwoofer());
    }

     ParallelRaceGroup segment = new ParallelRaceGroup();
    for (int i = 0; i < autoString.length() - 1; i++) {
      segment = new ParallelRaceGroup();
      char current = autoString.charAt(i);
      char next = autoString.charAt(i+1);

      if (current == 'A' && next == 'A') {
        setFeedback("Duck you Fylan");
      }

      try {
        // Load path
        if (!(next == current)) {
          PathPlannerPath path = PathPlannerPath.fromPathFile("" + current + "-" + next);
          trajectories.add(
              path.getTrajectory(
                new ChassisSpeeds(),
                path.getPreviewStartingHolonomicPose().getRotation()
              ));
          Command cmd = Commands.sequence(AutoBuilder.followPath(path), Commands555.waitForTime(0.2).until(RobotContainer.shooter::isNoteInTransport));
          segment = new ParallelRaceGroup(cmd);
        } else {
          setFeedback("Scoring Mode "); 
        }

        
      } catch (Exception e) {
        setFeedback("Path File Not Found");
        autoCommand = Commands.runOnce(() -> {});
        
      }
      boolean isGoingToNote = Array555.indexOf(AutoConstants.NOTES, next) != -1;
      boolean isFromCloseNote = current == 'A' || current == 'B' || current == 'C';
      
      boolean isFromNote = Array555.indexOf(AutoConstants.NOTES, current) != -1;
      boolean isGoingToFarNoteScoreLocation = next == '6' || next == '7';

      // If we're trying to intake, not score
      if (isGoingToNote && (!isFromNote || isFromCloseNote)) {
        segment.addCommands(Commands555.loadNote()); //A version of loadNote that ramps the shooter back up to speaker speed after (probably should be loadNoteAuto)
      } 
      
      finalPath.addCommands(segment);

      // Only rely on vision and driving forward manually if it's a close note.
      if (isGoingToNote) {
        finalPath.addCommands(Commands555.loadNoteAuto().onlyWhile(() -> {return !RobotContainer.shooter.isNoteInTransport();}).withTimeout(1.5));
      }

      // If we're trying to score
      if ((isFromNote && isGoingToNote && !isFromCloseNote) || (current == next) || (next == '5') || isGoingToFarNoteScoreLocation) { 
        Rotation2d angle = new Rotation2d();
        if (next == 'A') {
          angle = Rotation2d.fromDegrees(-30);
        }
        else if (next == 'B') {
          angle = Rotation2d.fromDegrees(0);
        }
        else if (next == 'C') {
          angle = Rotation2d.fromDegrees(26.57);
        }
        else if (next == '5') {
          angle = Rotation2d.fromDegrees(-26.57);
        }
        else if (next == '6') {
          angle = Rotation2d.fromDegrees(-35);
        }
        else if (next == '7') {
          angle = Rotation2d.fromDegrees(0);
        }


        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
          angle = GeometryUtil.flipFieldRotation(angle);
        } 
        if (next != '5' || next != '7') {
          finalPath.addCommands(Commands.parallel(Commands555.goToAngleFieldRelative(Drivetrain.wrapRotation(angle), false).withTimeout(0.9)), Commands555.setSprocketAngleWithStop(RobotContainer.shooterLimelight::bestFit)
          .withTimeout(1));
        }
        
        finalPath.addCommands(Commands555.scoreModeAuto().withTimeout(0.9));
        // finalPath.addCommands(Commands555.scoreAmp());
      }


      if (next == '4') { // Amp
        finalPath.addCommands(Commands555.scoreAmpAuto());
        
      } else if (next == '1' || next == '2' || next == '3') { // Up against subwoofer
        finalPath.addCommands(Commands555.scoreSubwoofer());
        
      }

      if (next == 'A' || next == 'B' || next == 'C' || next == '5') {
        // finalPath.addCommands(Commands555.addVisionMeasurement());
      }
      
    }
  
    setFeedback("Successfully Created Auto Sequence!");
    autoCommand = finalPath;
  }


}
  

