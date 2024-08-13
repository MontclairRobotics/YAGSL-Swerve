package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.vision.LimelightHelpers.RawFiducial;
import swervelib.SwerveDrive;
import swervelib.imu.SwerveIMU;

import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;



public class Limelight extends SubsystemBase {
  private double lastTx = 0;

  private String cameraName;
  private DetectionType defaultPipe;
  private Debouncer targetDebouncer =
      new Debouncer(VisionConstants.TARGET_DEBOUNCE_TIME, DebounceType.kFalling);
  private double priorityId = 0;

  public Limelight(String cameraName, DetectionType defaultPipe) {
    this.defaultPipe = defaultPipe;
    double[] camerapose_robotspace = new double[] {-1, -1, -1, 0.0, 0.0, 0.0};
    LimelightHelpers.setLimelightNTDoubleArray(
        cameraName, "camerapose_robotspace", camerapose_robotspace);

    this.cameraName = cameraName;
  }


  public double getTimestampSeconds() {
    double latency =
        (LimelightHelpers.getLimelightNTDouble(cameraName, "cl")
                + LimelightHelpers.getLimelightNTDouble(cameraName, "tl"))
            / 1000.0;

    return Timer.getFPGATimestamp() - latency;
  }


  public boolean hasValidTarget() {
    boolean hasMatch = (LimelightHelpers.getLimelightNTDouble(cameraName, "tv") == 1.0);
    return targetDebouncer.calculate(hasMatch);
    // return true;
  }

  public boolean currentPipelineMatches(DetectionType type) {

    int pipeline = (int) LimelightHelpers.getCurrentPipelineIndex(cameraName);
    return type.getPipe() == pipeline;
  }

  public double getObjectXSafe() {
    if (LimelightHelpers.getLimelightNTDouble(cameraName, "tv") != 1) {
      return lastTx;
    } else {
      lastTx = getObjectTX();
      return getObjectTX();
    }
  }

  /* 
   TODO: Make sure I didn't mess up this pipeline switcher. setCameraMode_Driver() and setCameraMode_Processor()
   were removed in 2024.6 LimelightHelpers so here's my fix! -JR

  */

  // public void setPipelineTo(DetectionType type) {
  //   if (type == DetectionType.DRIVER) {
  //     LimelightHelpers.setCameraMode_Driver(cameraName);
      
  //   } else { LimelightHelpers.setCameraMode_Processor(cameraName);
  //   }

  //   LimelightHelpers.setPipelineIndex(cameraName, type.getPipe());
  // }

  public void setPipelineTo(DetectionType type) {
    if (type == DetectionType.DRIVER) {
      setCameraMode(true);
    } else {
      setCameraMode(false);
    }

    LimelightHelpers.setPipelineIndex(cameraName, type.getPipe());

  }
  
  public void setCameraMode(boolean driverMode) {
    if (driverMode) {
      NetworkTableInstance.getDefault().getTable(cameraName).getEntry("camMode").setNumber(1);
    } else {
      NetworkTableInstance.getDefault().getTable(cameraName).getEntry("camMode").setNumber(0);
    }
    
  }

  

  @AutoLogOutput(key="Limelight-({cameraName})/TX")
  public double getObjectTX() {
    double tx = LimelightHelpers.getTX(cameraName);
     if (cameraName.equals("limelight-shooter")) {
      tx = -getHeadingToPriorityID();
    }
    return tx;
  }

  @AutoLogOutput(key="Limelight-({cameraName})/TY")
  public double getObjectTY() {
    return LimelightHelpers.getTY(cameraName);
  }
  
  public double getPipeline() {
    return LimelightHelpers.getCurrentPipelineIndex(cameraName);
  }

  
  public Pose2d getBotPose() {
    return LimelightHelpers.getBotPose2d_wpiBlue(cameraName);
  }

  // public Pose2d funBotPose() {
  //   return LimelightHelpers.toPose2D(LimelightHelpers.getBotPose_TargetSpace(cameraName));
  // }
  
  public DetectionType getPipelineType() {
    if (LimelightHelpers.getLimelightNTDouble(cameraName, "camMode") == 1) {
      return DetectionType.DRIVER;
    }
    return DetectionType.getType((int) getPipeline());
  }

  public Pose2d getTargetPoseRobotSpace() { //TODO did I screw this up?
    double[] doubleArr =  LimelightHelpers.getTargetPose_RobotSpace(cameraName);
    return new Pose2d(doubleArr[0], doubleArr[1], Rotation2d.fromDegrees(doubleArr[5]));
  }


  public void setPriorityTagID(int id) {
    LimelightHelpers.setPriorityTagID(cameraName, id);
  }
  
  
  public double getDistanceToSpeaker() {

    double distance =
        (VisionConstants.SPEAKER_APRILTAG_HEIGHT - VisionConstants.SHOOTER_LIMELIGHT_HEIGHT)
            / Math.tan(
                (Math.PI / 180.0)
                    * (VisionConstants.SHOOTER_LIMELIGHT_ANGLE_DEGREES + getVerticalToPriorityID()));

    return distance/Math.cos(getHeadingToPriorityID() * (Math.PI / 180));
  }

  //returns the component of distance in a straight wall from the lens of the limelight to the wall
  public double getStraightDistanceToSpeaker() {
    double distance =
        (VisionConstants.SPEAKER_APRILTAG_HEIGHT - VisionConstants.SHOOTER_LIMELIGHT_HEIGHT)
            / Math.tan(
                (Math.PI / 180.0)
                    * (VisionConstants.SHOOTER_LIMELIGHT_ANGLE_DEGREES + getVerticalToPriorityID()));

    return distance;

  }


  //returns adjusted heading to aim for the actual target instead of the april tag
  public Rotation2d maxIsStupid() {
    double distance = getStraightDistanceToSpeaker();
    double distanceNorm = getDistanceToSpeaker();
    double heading = Math.acos(distance / distanceNorm);
    double offsetFromTag = 6.0;
System.out.println(distance + " " + distanceNorm + " " + Math.acos(distance / distanceNorm) * (180/Math.PI)+" " + RobotContainer.drivetrain.getWrappedRotation().getDegrees());
    double angle = Math.asin((offsetFromTag * Math.sin(heading)) / (Math.sqrt(Math.pow(distance,2) + Math.pow(offsetFromTag,2) - 2 * distance * offsetFromTag * Math.cos(heading))));

    return Rotation2d.fromRadians(angle);

  }

  public void setDefaultPipeline() {
    // if (DriverStation.isTeleop()) {
    //   setPipelineTo(DetectionType.DRIVER);
    // } else {
    //   setPipelineTo(defaultPipe);
    // }
    setPipelineTo(defaultPipe);
  }

  public double getTXRaw() {
    return LimelightHelpers.getTX(cameraName);
  }

  @Override
  public void periodic() {
    
    // SwerveDrive drivetrain = RobotContainer.drivetrain.getSwerveDrive();

    // LimelightHelpers.SetRobotOrientation(cameraName, 
    // drivetrain.getOdometryHeading().getDegrees(), 
    // 0, 0, 0, 0, 0);

    // PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    
    // double angularVelocity = drivetrain.getFieldVelocity().omegaRadiansPerSecond;

    // if (Math.abs(angularVelocity) <= VisionConstants.POSE_ANGULAR_VELOCITY_CUTOFF || estimate.tagCount == 0) {
    //   drivetrain.addVisionMeasurement(estimate.pose, Timer.getFPGATimestamp(), VisionConstants.IDEAL_VISION_STD_DEVS);
    // }
     
    // }
    

    if (DriverStation.isDisabled()) {
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        LimelightHelpers.setPriorityTagID(cameraName, 4); //4
        priorityId = 4;
      } else {
        LimelightHelpers.setPriorityTagID(cameraName, 7);
        priorityId = 7;
      }
    }

  }


  


  public double getHeadingToPriorityID() {
    LimelightHelpers.RawFiducial[] tagArr = getAdjustedPose().rawFiducials;
   tagArr = Arrays.stream(tagArr).filter((entry) -> {return entry.id == priorityId;}).toArray(LimelightHelpers.RawFiducial[]::new);

   if (tagArr.length == 1) {
    // System.out.println(tagArr[0].txnc);
    return -tagArr[0].txnc + 1.5;
   // }
   }
   return 0;
 }

 public double getVerticalToPriorityID() {
    LimelightHelpers.RawFiducial[] tagArr = getAdjustedPose().rawFiducials;
   tagArr = Arrays.stream(tagArr).filter((entry) -> {return entry.id == priorityId;}).toArray(LimelightHelpers.RawFiducial[]::new);

   if (tagArr.length == 1) {
    // System.out.println(tagArr[0].tync+0.6);
    return tagArr[0].tync+0.6;
   // }
   }
   return 0;
 }

  //degrees
  public double getAngleToSpeaker() {
    LimelightHelpers.PoseEstimate botPose = getAdjustedPose();
    Pose2d odomPose = RobotContainer.drivetrain.getSwerveDrive().getPose();
    // if (botPose.tagCount < 2 || botPose.avgTagDist > 4) {
    LimelightHelpers.RawFiducial[] tagArr = botPose.rawFiducials;
    tagArr = Arrays.stream(tagArr).filter((entry) -> {return entry.id == priorityId;}).toArray(LimelightHelpers.RawFiducial[]::new);
    if (tagArr.length == 1) {
      return -tagArr[0].txnc + RobotContainer.shooterLimelight.maxIsStupid().getDegrees();
      // }
    }

    double verticalDistance = 5.55-odomPose.getY();
    double horizontalDistance = odomPose.getX() - 0.1524; //6 inches in meters

    if (DriverStation.isAutonomous()) {
      horizontalDistance = 16.54 - odomPose.getX();
    }

    double rotation = 180 + (180 / Math.PI) * Math.atan(verticalDistance/horizontalDistance); //180 + because we are on the back of robot

    if (DriverStation.isAutonomous()) {
      rotation = Drivetrain.wrapRotation(Rotation2d.fromDegrees(180 - rotation - RobotContainer.drivetrain.getWrappedRotation().getDegrees())).getDegrees();
    }
    return rotation;
    


    

  }

    public double getPoseDistanceToSpeaker() {
    LimelightHelpers.PoseEstimate botPose = getAdjustedPose();
    Pose2d odomPose = RobotContainer.drivetrain.getSwerveDrive().getPose();
    // if (botPose.tagCount < 2 || botPose.avgTagDist > 4) {
    LimelightHelpers.RawFiducial[] tagArr = botPose.rawFiducials;
    tagArr = Arrays.stream(tagArr).filter((entry) -> {return entry.id == priorityId;}).toArray(LimelightHelpers.RawFiducial[]::new);
    if (tagArr.length == 1) {
      getDistanceToSpeaker();
      // }
    }

    double verticalDistance = 5.55-odomPose.getY();
    double horizontalDistance = odomPose.getX() - 0.1524; //6 inches in meters

    if (DriverStation.isAutonomous()) {
      horizontalDistance = 16.54 - odomPose.getX();
    }

    double distance = Math.sqrt(Math.pow(verticalDistance, 2) + Math.pow(horizontalDistance, 2));

    return distance;
    


    

  }
  
  public double bestFit() {
    // double  
    // return (0.001717 * (Math.pow(x, 2))) + (-0.6251 * x) + (83.41);
    return bestFitFromDistance(getDistanceToSpeaker()) - 6.5;
  }

  public double bestFitFromDistance(double x) {
    return (59.41*Math.exp(-0.01694*x)) + 27.01;
  }

  public double getSpeedForSpeaker() {
    return ShooterConstants.SPEAKER_EJECT_SPEED - 20 * Math.abs(90-RobotContainer.drivetrain.getWrappedRotation().getDegrees());
  }


  public LimelightHelpers.PoseEstimate getAdjustedPose() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red && DriverStation.isTeleop()) {
      return LimelightHelpers.getBotPoseEstimate_wpiRed(cameraName);
    }
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
  }

  @AutoLogOutput(key="Limelight-({cameraName})/Is_Aligned")
  public boolean isAligned() {
    double tx = getObjectTX();
    return Math.abs(tx) < DriveConstants.ANGLE_DEADBAND;
  }


}

