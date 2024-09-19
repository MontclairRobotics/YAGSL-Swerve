package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.util.BreakBeam;

// Drive by scoring angle calculation is: arctan(height/distance);

public class Shooter extends SubsystemBase {

  // The motors
  public final CANSparkMax topMotor =
      new CANSparkMax(Ports.SHOOTER_TOP_MOTOR, MotorType.kBrushless);
  public final CANSparkMax bottomMotor =
      new CANSparkMax(Ports.SHOOTER_BOTTOM_MOTOR, MotorType.kBrushless);
  public final CANSparkMax transportMotor =
      new CANSparkMax(Ports.SHOOTER_MOTOR_TRANSPORT, MotorType.kBrushless);

  // The encoders on each motor
  private RelativeEncoder topEncoder = topMotor.getEncoder();
  private RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
  private RelativeEncoder transportEncoder = transportMotor.getEncoder();

  // PID controllers on each SparkMax
  private SparkPIDController topController = topMotor.getPIDController();
  private SparkPIDController bottomController = bottomMotor.getPIDController();
  private SparkPIDController transportController = transportMotor.getPIDController();

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
  private SimpleMotorFeedforward transportMotorFeedforward =
      new SimpleMotorFeedforward(
          Constants.ShooterConstants.TRANSPORT_FF_KS,
          Constants.ShooterConstants.TRANSPORT_FF_KV,
          Constants.ShooterConstants.TRANSPORT_FF_KA);



  private boolean isShooting = false;
  private boolean isTransporting = false;

  private BreakBeam breakBeam = new BreakBeam(Ports.TRANSPORT_BEAM_BREAK, ShooterConstants.BEAM_BRAKE_INVERT);

  private double targetTopSpeed = 0;
  private double targetBottomSpeed = 0;

  Debouncer m_debouncer = new Debouncer(0.3, Debouncer.DebounceType.kRising);

  public Shooter() {
    // Shuffleboard.getTab("Debug").addDouble("Top velocity", () -> {return topEncoder.getVelocity();});
    // Shuffleboard.getTab("Debug").addDouble("Bottom velocity", () -> {return bottomEncoder.getVelocity();});
    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();
    transportMotor.restoreFactoryDefaults();

    topMotor.setInverted(false);
    bottomMotor.setInverted(false);
    transportMotor.setInverted(true);
    

    topController.setP(Constants.ShooterConstants.TOP_SHOOTER_PID_KP, 1);
    topController.setI(Constants.ShooterConstants.TOP_SHOOTER_PID_KI, 1);
    topController.setD(Constants.ShooterConstants.TOP_SHOOTER_PID_KD, 1);
    topController.setOutputRange(-1, 1, 1);

    bottomController.setP(Constants.ShooterConstants.BOTTOM_SHOOTER_PID_KP, 1);
    bottomController.setI(Constants.ShooterConstants.BOTTOM_SHOOTER_PID_KI, 1);
    bottomController.setD(Constants.ShooterConstants.BOTTOM_SHOOTER_PID_KD, 1);
    bottomController.setOutputRange(-1, 1);

    transportController.setP(Constants.ShooterConstants.TRANSPORT_PID_KP, 1);
    transportController.setI(Constants.ShooterConstants.TRANSPORT_PID_KI, 1);
    transportController.setD(Constants.ShooterConstants.TRANSPORT_PID_KD, 1);
    transportController.setOutputRange(-1, 1);
    
    // Shuffleboard.getTab("Debug").addBoolean("Transport Beam Break", () -> {
    //   return isNoteInTransport();
    // });

    transportMotor.setIdleMode(IdleMode.kBrake);
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

    topController.setReference(topVelocity, ControlType.kVelocity, 1, topFeedForward);
    bottomController.setReference(bottomVelocity, ControlType.kVelocity, 1, bottomFeedForward);

    isShooting = true;
  }

  
  /** Runs transport at given velocity */
  public void transportVelocity(double velocity) {
    System.out.println("transportVelocity: " + velocity);

    double transportFeedForward = transportMotorFeedforward.calculate(velocity);
    System.out.println("TranportFF: " + transportFeedForward);

    transportController.setReference(velocity, ControlType.kVelocity, 1, transportFeedForward);

    isTransporting = true;
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

  /** Shoots (used for speaker) */
  public void shootSpeaker() {
    topMotor.set(ShooterConstants.SPEAKER_EJECT_SPEED);
    bottomMotor.set(ShooterConstants.SPEAKER_EJECT_SPEED);
    
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
    final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    final MutableMeasure<Angle> rotations = mutable(Revolutions.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    final MutableMeasure<Velocity<Angle>> velocity = mutable(RevolutionsPerSecond.of(0));

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
                (Measure<Voltage> volts) -> {
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
