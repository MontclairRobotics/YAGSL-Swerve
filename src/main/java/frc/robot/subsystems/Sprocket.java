package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import frc.robot.Constants.*;

import org.littletonrobotics.junction.AutoLogOutput;

public class Sprocket extends SubsystemBase {

  // Construct two CANSparkMAX motor controllers
  private final SparkMax leftMotor =
      new SparkMax(Ports.LEFT_ANGLE_MOTOR, MotorType.kBrushless);
  private final SparkMax rightMotor =
      new SparkMax(Ports.RIGHT_ANGLE_MOTOR, MotorType.kBrushless);


  private double targetSpeed; // Tracks the target speed of the arm before safeties

  //The way this is implemented sucks, should probably be handled by commands.
  private boolean usingPID; // Tracks whether the arm is in manual or automatic control (PID)

  // A controller which is responsible for automatic movement
  private PIDController pidController =
      new PIDController(
          ArmConstants.angleKP.get(), ArmConstants.angleKI.get(), ArmConstants.angleKD.get());
  // Tracks the voltage to be outputted to the motor when using automatic control
  private double pidVoltage;


  // Creates a new encoder object to represent the REV Throughbore encoder on the arm
  public DutyCycleEncoder absEncoder = new DutyCycleEncoder(Ports.SPROCKET_ABS_ENCODER, 360, 0);
  

  // Constructs the subsystem
  public Sprocket() {
    
    //Configure the motors with inverts, and tells it to brake when stopping
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
      .inverted(LEFT_INVERT.get())
      .idleMode(IdleMode.kBrake);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
      .inverted(RIGHT_INVERT.get())
      .idleMode(IdleMode.kBrake);

    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Code responsible for logging important values to our driver dashboard, Shuffleboard

    // Shuffleboard.getTab("Debug").addBoolean("Using PID", () -> usingPID);
    Shuffleboard.getTab("Debug").addDouble("Sprocket Encoder", () -> getEncoderPosition());
    Shuffleboard.getTab("Debug").addDouble("Sprocket Encoder Raw", () -> getRawPosition());

    // Don't worry about this, it updates the PID constants/gains when they are changed
    // on the dashboard
    angleKD.whenUpdate(
        (k) -> {
          pidController.setD(k);
        });
    angleKI.whenUpdate(
        (k) -> {
          pidController.setI(k);
        });
    angleKP.whenUpdate(
        (k) -> {
          pidController.setP(k);
        });
    
    // Sets a tolerance for the automated motion
    pidController.setTolerance(SPROCKET_ANGLE_DEADBAND);
    
  }

  // Sets the target speed. This probably doesn't need to be its own method.
  public void setSpeed(double speed) {
    targetSpeed = speed;
    if (targetSpeed != 0) usingPID = false;
  }

  // Takes input from the controller, there are probably better ways to do this
  // See line 81 in RobotContainer.java to see where this is called.
  public void setInputFromJoystick(CommandPS5Controller controller) {

    // Invert the output of the joystick [-1, 1] and add a deadzone to account for stick drift
    double yAxis = -MathUtil.applyDeadband(controller.getLeftY(), 0.05);

    // If the joystick isn't being moved and we're not running automatically, stop moving the arm
    if (yAxis == 0.0 && !usingPID) {
      stop();
    }

    // otherwise, update the target speed.
    setSpeed(yAxis);
  }

  // Tells the arm to move automatically to an angle
  public void setPosition(Rotation2d angle) {
    usingPID = true;
    pidController.setSetpoint(angle.getDegrees());
  }

  /** Stop sprocket */
  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  /** 
   * Returns whether or not the arm is safe to move in the target direction without breaking
   */
  @AutoLogOutput(key = "Sprocket/IsSprocketSafe")
  public boolean isSprocketSafe() {
    // return true;
    boolean goingUp = false;

    // Are we trying to move the arm upward or downward
    if (usingPID) {
      goingUp = pidVoltage > 0;
    } else {
      goingUp = targetSpeed > 0;
    }

    // If we're close to the minimum safe angle and going down
    // Or close to the maximum safe angle and going up, it's not safe to move
    if ((getEncoderPosition() < ENCODER_MIN_ANGLE + SPROCKET_ANGLE_LIMIT_DEADBAND && !goingUp)
        || (getEncoderPosition() > ENCODER_MAX_ANGLE - SPROCKET_ANGLE_LIMIT_DEADBAND && goingUp)) {
      return false;
    }
    return true;
  }
  /**
   * Gets the position of the sensor.
   * It sometimes initializes to a number a random number of rotations
   * away. Our code needs it to be between 0 and 360 degrees or the arm
   * will break. This code here adds/subtracts 360 until it is in the correct
   * range. We don't know why the sensor does this.
   * @return The adjusted sensor angle
   */
  @AutoLogOutput(key = "Sprocket/EncoderAngle")
  public double getEncoderPosition() {
    double pos = getRawPosition();
    pos = pos % 360;
    if (pos < 0) {
      return pos + 360;
    }
    return pos;
  }

  /**
   * This returns the position the sensor reports, after we offset it so it is aligned
   * with the horizontal. This offsetting should be moved to the constructor
   * with the new API in 2025.
   * @return
   */
  @AutoLogOutput(key = "Sprocket/EncoderRawAngle")
  public double getRawPosition() {
    return ((absEncoder.get())-49.2); //* ((double) 14/64)) + 79;//76;
  }

  /**
   * @return Whether the arm is at its setpoint (target position)
   */
  @AutoLogOutput(key = "Sprocket/IsAtAngle")
  public boolean isAtAngle() {
    return pidController.atSetpoint();
  }

  //Runs every frame
  @Override
  public void periodic() {

    // Calculate the new output of the motion controller if the arm is at a given position
    // Basically, you tell it where it is, it works out how close it is to the set target
    // then outputs a voltage for the motor.
    pidVoltage = pidController.calculate(getEncoderPosition());

    // Don't move unless the arm is safe to do so
    if (isSprocketSafe()) {
      if (!usingPID) { //if moving manually
        leftMotor.set(targetSpeed); //set to target manual speed
        rightMotor.set(targetSpeed);
      } else { //otherwise set to calculated voltage divided by total to get a speed between 0 and 1
        //This should be .setVoltage without dividing by the bus voltage, it was giving us problems
        leftMotor.set(pidVoltage / leftMotor.getBusVoltage());
        rightMotor.set(pidVoltage / rightMotor.getBusVoltage());
      }
    } else {
      stop(); // If we're not safe stop moving
    }

  }

  // Don't worry about this just yet. It helps us configure the automatic controller.
  public SysIdRoutine getSysId() {
    MutVoltage appliedVoltage = Units.Volts.mutable(0);
    MutAngle degrees = Units.Degrees.mutable(0);
    MutAngularVelocity motorVelocity =
        Units.DegreesPerSecond.mutable(0);

    return new SysIdRoutine(
        new Config(),
        new Mechanism(
            (Voltage volts) -> {
              leftMotor.setVoltage(volts.in(Units.Volts));
              rightMotor.setVoltage(volts.in(Units.Volts));
            },
            (SysIdRoutineLog log) -> {
              log.motor("Left")
                  .voltage(
                      appliedVoltage.mut_replace(
                          leftMotor.getAppliedOutput() * leftMotor.getBusVoltage(), Units.Volts))
                  .angularPosition(degrees.mut_replace(getEncoderPosition(), Units.Degrees))
                  .angularVelocity(
                      motorVelocity.mut_replace(
                          leftMotor.getEncoder().getVelocity(), Units.DegreesPerSecond));

              log.motor("Right")
                  .voltage(
                      appliedVoltage.mut_replace(
                          rightMotor.getAppliedOutput() * rightMotor.getBusVoltage(), Units.Volts))
                  .angularPosition(degrees.mut_replace(getEncoderPosition(), Units.Degrees))
                  .angularVelocity(
                      motorVelocity.mut_replace(
                          rightMotor.getEncoder().getVelocity(), Units.DegreesPerSecond));
            },
            this,
            "Sprocket"));
  }
}
