package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.util.BreakBeam;

// Drive by scoring angle calculation is: arctan(height/distance);

public class Shooter extends SubsystemBase {

  // The motors
  public final SparkMax topMotor =
      new SparkMax(Ports.SHOOTER_TOP_MOTOR, MotorType.kBrushless);
  public final SparkMax bottomMotor =
      new SparkMax(Ports.SHOOTER_BOTTOM_MOTOR, MotorType.kBrushless);
  public final SparkMax transportMotor =
      new SparkMax(Ports.SHOOTER_MOTOR_TRANSPORT, MotorType.kBrushless);

  // The encoders on each motor
  private RelativeEncoder topEncoder = topMotor.getEncoder();
  private RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
  private RelativeEncoder transportEncoder = transportMotor.getEncoder();

  // PID controllers on each SparkMax
  private SparkClosedLoopController topController;
  private SparkClosedLoopController bottomController;

  // Simple FeedForward calculations for each motor
  private SimpleMotorFeedforward topMotorFeedforward =
      new SimpleMotorFeedforward(
          Constants.ShooterConstants.TOP_SHOOTER_FF_KS,
          Constants.ShooterConstants.TOP_SHOOTER_FF_KV,
          Constants.ShooterConstants.TOP_SHOOTER_FF_KA);
  private SimpleMotorFeedforward bottomMotorFeedforward =
      new SimpleMotorFeedforward(
          Constants.ShooterConstants.BOTTOM_SHOOTER_FF_KS,
          Constants.ShooterConstants.BOTTOM_SHOOTER_FF_KV,
          Constants.ShooterConstants.BOTTOM_SHOOTER_FF_KA);



  private boolean isShooting = false;
  private boolean isTransporting = false;

  private BreakBeam breakBeam = new BreakBeam(Ports.TRANSPORT_BEAM_BREAK, ShooterConstants.BEAM_BRAKE_INVERT);

  private double targetTopSpeed = 0;
  private double targetBottomSpeed = 0;

  public Shooter() {
    // Shuffleboard.getTab("Debug").addDouble("Top velocity", () -> {return topEncoder.getVelocity();});
    // Shuffleboard.getTab("Debug").addDouble("Bottom velocity", () -> {return bottomEncoder.getVelocity();});
    SparkMaxConfig topConfig = new SparkMaxConfig();
    topConfig
      .inverted(false)
      .idleMode(IdleMode.kCoast);
    topConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(Constants.ShooterConstants.TOP_SHOOTER_PID_KP, Constants.ShooterConstants.TOP_SHOOTER_PID_KI, Constants.ShooterConstants.TOP_SHOOTER_PID_KD)
      .outputRange(-1, 1);


    SparkMaxConfig bottomConfig = new SparkMaxConfig();
    bottomConfig
      .inverted(false)
      .idleMode(IdleMode.kCoast);
    bottomConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(Constants.ShooterConstants.BOTTOM_SHOOTER_PID_KP, Constants.ShooterConstants.BOTTOM_SHOOTER_PID_KI, Constants.ShooterConstants.BOTTOM_SHOOTER_PID_KD)
      .outputRange(-1, 1);

    SparkMaxConfig transportConfig = new SparkMaxConfig();
    transportConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake);

    topMotor.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomMotor.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    transportMotor.configure(transportConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    topController = topMotor.getClosedLoopController();
    bottomController = bottomMotor.getClosedLoopController();

  } 

  /** Is shooting running? */
  public boolean isShooting() {
    return isShooting;
  }

  /** Is transport running? */
  
  public boolean isTransporting() {
    return isTransporting;
  }

  /** Runs both top and bottom shooter motors at the same veloctiy */
  public void shootVelocity(double velocity) {
    shootVelocity(velocity, velocity);
  }

  /** Runs top and bottom shooter motors at different velocties */
  public void shootVelocity(double topVelocity, double bottomVelocity) {
    targetTopSpeed = topVelocity;
    targetBottomSpeed = bottomVelocity;
    
    System.out.println("shootVelocity: " + topVelocity + " " + bottomVelocity);

    double topFeedForward = topMotorFeedforward.calculate(topVelocity);
    double bottomFeedForward = bottomMotorFeedforward.calculate(bottomVelocity);
    System.out.println("TopFF: " + topFeedForward + " BottomFF: " + bottomFeedForward);

    topController.setReference(topVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, topFeedForward);
    bottomController.setReference(bottomVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, bottomFeedForward);

    isShooting = true;
  }


  @AutoLogOutput(key = "Shooter/IsAtSpeed")
  public boolean isAtSpeed() {
    double currentTopSpeed = topEncoder.getVelocity();
    double currentBottomSpeed = bottomEncoder.getVelocity();
    if (Math.abs(targetTopSpeed - currentTopSpeed) > ShooterConstants.VELOCITY_DEADBAND) {
      return false;
    }
    if (Math.abs(targetBottomSpeed - currentBottomSpeed) > ShooterConstants.VELOCITY_DEADBAND) {
      return false;
    }
    return true;
  }
 

  /** Stops the shooter */
  public void stopShooter() {
    targetTopSpeed = 0;
    targetBottomSpeed = 0;
    topMotor.stopMotor();
    bottomMotor.stopMotor();
    isShooting = false;
  }

  /** Stops the transport */
  public void stopTransport() {
    transportMotor.stopMotor();
    isTransporting = false;
  }

  public void shootActually(double topSpeed, double bottomSpeed) {
    targetTopSpeed = topSpeed;
    targetBottomSpeed = bottomSpeed;

    topMotor.set(-topSpeed);
    bottomMotor.set(bottomSpeed);
   
  }

  // public void translateToAmp(double forwardVelocity){
  //   Debouncer hasEnded = new Debouncer(debounceTime:0.1, DebounceTpe.k)
  // }
  

  public void transportStart(double transportSpeed) {
    transportMotor.set(transportSpeed);
  }

  public double getTopVelocity() {
    return topEncoder.getVelocity();
  }

  public double getBottomVelocity() {
    return bottomEncoder.getVelocity();
  }

  /** Shoots (used for amp) */
  public void shootAmp() {
    topMotor.set(ShooterConstants.AMP_EJECT_SPEED_TOP);
    bottomMotor.set(ShooterConstants.AMP_EJECT_SPEED_BOTTOM);
  }

  /** Stops the motors */
  public void stop() {
    stopShooter();
  }

  /** reverse shooter, in case shooter jams, etc. */
  public void reverseShooter() {
    topMotor.set(-ShooterConstants.SPEAKER_EJECT_SPEED);
    bottomMotor.set(-ShooterConstants.SPEAKER_EJECT_SPEED);
  }

  @AutoLogOutput(key = "Shooter/isNoteInTransport")
  public boolean isNoteInTransport() {

    return breakBeam.get();
  }

  public void toggleShooter() {
    if (isShooting) {
      stopShooter();
    } else {
      shootVelocity(ShooterConstants.SPEAKER_EJECT_SPEED);
    }
  }

  

  public Command shooterSysId(String type, String motors, SysIdRoutine.Direction direction) {
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    final MutVoltage appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    final MutAngle rotations = Revolutions.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    final MutAngularVelocity velocity = RevolutionsPerSecond.mutable(0);

    boolean runTop = motors.contains("top");
    boolean runBottom = motors.contains("bottom");
    boolean runTransport = motors.contains("transport");

    // Create a new SysId routine for characterizing the drive.
    final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motors.
                (Voltage volts) -> {
                  if (runTop) {
                    topMotor.setVoltage(volts.in(Volts));
                  }
                  if (runBottom) {
                    bottomMotor.setVoltage(volts.in(Volts));
                  }
                  if (runTransport) {
                    transportMotor.setVoltage(volts.in(Volts));
                  }
                },
                // Tell SysId how to record data of each motor
                log -> {
                  if (runTop) {
                    // Record data for top motor
                    log.motor("top")
                        .voltage(
                            appliedVoltage.mut_replace(
                                topMotor.getAppliedOutput() * topMotor.getBusVoltage(), Volts))
                        .angularPosition(rotations.mut_replace(topEncoder.getPosition(), Rotations))
                        .angularVelocity(
                            velocity.mut_replace(topEncoder.getVelocity(), RotationsPerSecond));
                  }
                  if (runBottom) {
                    // Record data for bottom motor
                    log.motor("bottom")
                        .voltage(
                            appliedVoltage.mut_replace(
                                bottomMotor.getAppliedOutput() * bottomMotor.getBusVoltage(),
                                Volts))
                        .angularPosition(
                            rotations.mut_replace(bottomEncoder.getPosition(), Rotations))
                        .angularVelocity(
                            velocity.mut_replace(bottomEncoder.getVelocity(), RotationsPerSecond));
                  }
                  if (runTransport) {
                    log.motor("transport")
                        .voltage(
                            appliedVoltage.mut_replace(
                                transportMotor.getAppliedOutput() * topMotor.getBusVoltage(),
                                Volts))
                        .angularPosition(
                            rotations.mut_replace(transportEncoder.getPosition(), Rotations))
                        .angularVelocity(
                            velocity.mut_replace(
                                transportEncoder.getVelocity(), RotationsPerSecond));
                  }
                },
                // We are doing SysId on "this" subsystem
                this));

    if (type == "quasistatic") {
      return sysIdRoutine.quasistatic(direction);
    } else {
      return sysIdRoutine.dynamic(direction);
    }
  }

  // private int count = 0;

  @Override
  public void periodic() {

  
    // RobotContainer.operatorController.getHID().setRumble(RumbleType.kLeftRumble, 1);
    // RobotContainer.testController.setRumble(RumbleType.kBothRumble, 1);
  }

  public void teleopInit() {
    stop();
  }
}
