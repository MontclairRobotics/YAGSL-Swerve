package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.*;


import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

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

  private final CANSparkMax leftMotor =
      new CANSparkMax(Ports.LEFT_ANGLE_MOTOR, MotorType.kBrushless);
  private final CANSparkMax rightMotor =
      new CANSparkMax(Ports.RIGHT_ANGLE_MOTOR, MotorType.kBrushless);
  private InterpolatingDoubleTreeMap angleMap;
  private InterpolatingDoubleTreeMap speedMap;



  private double targetSpeed;
  private boolean usingPID;

  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;

  private PIDController pidController =
      new PIDController(
          ArmConstants.angleKP.get(), ArmConstants.angleKI.get(), ArmConstants.angleKD.get());
  private double pidVoltage;

  public DutyCycleEncoder absEncoder = new DutyCycleEncoder(Ports.SPROCKET_ABS_ENCODER);
  

  public Sprocket() {

    leftMotor.setInverted(LEFT_INVERT.get());
    rightMotor.setInverted(RIGHT_INVERT.get());
    // Shuffleboard.getTab("Debug").addBoolean("Using PID", () -> usingPID);
    Shuffleboard.getTab("Debug").addDouble("Sprocket Encoder", () -> getEncoderPosition());
    Shuffleboard.getTab("Debug").addDouble("Sprocket Encoder Raw", () -> getRawPosition());
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

    // Shuffleboard.getTab("Debug").addDouble("Limelight Distance", () -> {
    //   return RobotContainer.shooterLimelight.getDistanceToSpeaker();
    // });


    leftEncoder = leftMotor.getEncoder();
    leftEncoder.setPositionConversionFactor(1 / SPROCKET_ROTATIONS_PER_DEGREE);
    leftEncoder.setVelocityConversionFactor(1 / SPROCKET_ROTATIONS_PER_DEGREE * (1 / 60));
    leftEncoder.setPosition(ENCODER_MIN_ANGLE);

    rightEncoder = rightMotor.getEncoder();
    rightEncoder.setPositionConversionFactor(1 / SPROCKET_ROTATIONS_PER_DEGREE);
    leftEncoder.setVelocityConversionFactor(1 / SPROCKET_ROTATIONS_PER_DEGREE * (1 / 60));
    rightEncoder.setPosition(ENCODER_MIN_ANGLE);

   

    

    absEncoder.setDistancePerRotation(360.0);
    pidController.setTolerance(SPROCKET_ANGLE_DEADBAND);

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    //     Shuffleboard.getTab("Debug").addDouble("Current Distance", () -> {
    //   return getEncoderPosition();
    // });

    
  }

  public void setSpeed(double speed) {
    targetSpeed = speed;
    if (targetSpeed != 0) usingPID = false;
  }

  public void setInputFromJoystick(CommandPS5Controller controller) {
    double yAxis = -MathUtil.applyDeadband(controller.getLeftY(), 0.05);

    if (yAxis == 0.0 && !usingPID) {
      stop();
    }
    setSpeed(yAxis);
  }

  public void setPosition(Rotation2d angle) {
    usingPID = true;
    pidController.setSetpoint(angle.getDegrees());
  }

  /** Stop sprocket */
  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  @AutoLogOutput(key = "Sprocket/IsSprocketSafe")
  public boolean isSprocketSafe() {
    // return true;
    boolean goingUp = false;

    if (usingPID) {
      goingUp = pidVoltage > 0;
    } else {
      goingUp = targetSpeed > 0;
    }

    if ((getEncoderPosition() < ENCODER_MIN_ANGLE + SPROCKET_ANGLE_LIMIT_DEADBAND && !goingUp)
        || (getEncoderPosition() > ENCODER_MAX_ANGLE - SPROCKET_ANGLE_LIMIT_DEADBAND && goingUp)) {
      return false;
    }
    return true;
  }

  // Distance in meters, return in degrees
  public double calculateSpeed(double distance) {
    return speedMap.get(distance);
  }

  // Distance in meters, return in rpm
  public double calculateAngle(double distance) {
    return angleMap.get(distance);
  }
  
  @AutoLogOutput(key = "Sprocket/EncoderAngle")
  public double getEncoderPosition() {
    double pos = getRawPosition();
    pos = pos % 360;
    if (pos < 0) {
      return pos + 360;
    }
    return pos;
  }

  @AutoLogOutput(key = "Sprocket/EncoderRawAngle")
  public double getRawPosition() {
    return ((absEncoder.getDistance())-49.2); //* ((double) 14/64)) + 79;//76;
  }

  @AutoLogOutput(key = "Sprocket/IsAtAngle")
  public boolean isAtAngle() {
    return pidController.atSetpoint();
  }

  @Override
  public void periodic() {

    pidVoltage = pidController.calculate(getEncoderPosition());

    if (isSprocketSafe()) {
      if (!usingPID) {
        leftMotor.set(targetSpeed);
        rightMotor.set(targetSpeed);
      } else {
        leftMotor.set(pidVoltage / leftMotor.getBusVoltage());
        rightMotor.set(pidVoltage / rightMotor.getBusVoltage());
      }
    } else {
      stop();
    }

  }

  public SysIdRoutine getSysId() {
    MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
    MutableMeasure<Angle> degrees = MutableMeasure.mutable(Units.Degrees.of(0));
    MutableMeasure<Velocity<Angle>> motorVelocity =
        MutableMeasure.mutable(Units.DegreesPerSecond.of(0));

    return new SysIdRoutine(
        new Config(),
        new Mechanism(
            (Measure<Voltage> volts) -> {
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
