package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.*;
import frc.robot.util.LimitSwitch;

public class Climbers extends SubsystemBase {
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;

  private final LimitSwitch leftLimit;
  private final LimitSwitch rightLimit;
  private final RelativeEncoder rightEncoder;
  private final RelativeEncoder leftEncoder;
  
  //public boolean inMotion = false; 
  public boolean leftMovingUp = false;
  public boolean leftAtBottom = false;
  public boolean leftAtTop = false;
  private double leftSpeed = 0;

  public boolean rightMovingUp = false;
  public boolean rightAtBottom = false;
  public boolean rightAtTop = false;
  private double rightSpeed = 0;

  /** Creates objects for the motors, limits, and encoders Then sets the encoders */
  public Climbers() {
    leftMotor = new CANSparkMax(Ports.CLIMBER_LEFT_MOTOR, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Ports.CLIMBER_RIGHT_MOTOR, MotorType.kBrushless);

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    leftLimit = new LimitSwitch(Ports.CLIMBER_LEFT_LIMIT_SWITCH_PORT, true);
    rightLimit = new LimitSwitch(Ports.CLIMBER_RIGHT_LIMIT_SWITCH_PORT, true);
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    // leftEncoder.setPositionConversionFactor(
    //     ClimberConstants.ROTATIONS_PER_INCH); // Converts to inches per rotation
    // rightEncoder.setPositionConversionFactor(ClimberConstants.ROTATIONS_PER_INCH);

    // leftEncoder.setVelocityConversionFactor(1 / ClimberConstants.ROTATIONS_PER_INCH);
    // rightEncoder.setVelocityConversionFactor(1 / ClimberConstants.ROTATIONS_PER_INCH);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    // Shuffleboard.getTab("Debug").addDouble("Climber Height", () -> getHeight());
  //   Shuffleboard.getTab("Debug").addBoolean("At Top", () -> atTop());
  //   Shuffleboard.getTab("Debug").addBoolean("At Bottom", () -> atBottom());

    // Shuffleboard.getTab("Debug").addDouble("Left Encoder", () -> leftEncoder.getPosition());
    // Shuffleboard.getTab("Debug").addDouble("Right Encoder", () -> rightEncoder.getPosition());
    // Shuffleboard.getTab("Debug").addBoolean("Left Limit", () -> leftLimit.get());
    // Shuffleboard.getTab("Debug").addBoolean("Right Limit", () -> rightLimit.get());

  }



  /** Makes the climber go up */
  public void up() {
    if (!(rightLimit.get() && rightEncoder.getPosition() > 7)) {
      rightMotor.set(-ClimberConstants.CLIMBER_SPEED);
    } else {
      rightMotor.set(0);
    }
    if (!(leftLimit.get() && leftEncoder.getPosition() > 7)) {
      leftMotor.set(-ClimberConstants.CLIMBER_SPEED);
    } else {
      leftMotor.set(0);
    }
  }

  public void leftUp() {
    leftMotor.set(ClimberConstants.CLIMBER_SPEED);
  }

  public void leftDown() {
    leftMotor.set(-ClimberConstants.CLIMBER_SPEED);
  }

  public void leftStop() {
    leftMotor.stopMotor();
  }

  public void rightUp() {
    rightMotor.set(ClimberConstants.CLIMBER_SPEED);
  }

  public void rightDown() {
    rightMotor.set(-ClimberConstants.CLIMBER_SPEED);
  }

  public void rightStop() {
    rightMotor.stopMotor();
  }

  public double getHeight() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition())/2;
  }
  /** Climber arm goes down */
  public void down() {
    if (!(rightLimit.get() && rightEncoder.getPosition() < 7)) {
      rightMotor.set(ClimberConstants.CLIMBER_SPEED);
    } else {
      rightMotor.set(0);
    } 
    if (!(leftLimit.get() && leftEncoder.getPosition() < 7)) {
      leftMotor.set(ClimberConstants.CLIMBER_SPEED);
    } else {
      leftMotor.set(0);
    }

  }

  public void setInputFromController(CommandPS5Controller controller) {
    double yAxis = -MathUtil.applyDeadband(controller.getRightY(), 0.05);

    leftSpeed = yAxis;
    rightSpeed = yAxis;
    // leftMotor.set(-yAxis);
    // rightMotor.set(-yAxis);

    // if (yAxis == 0.0) {
    //   rightSpeed = 0;
    //   leftSpeed = 0;
    // }


  }

  public void setInputFromSticks(CommandPS5Controller controller) {
    leftSpeed = -MathUtil.applyDeadband(controller.getLeftY(), 0.05);
    rightSpeed = -MathUtil.applyDeadband(controller.getRightY(), 0.05);
  }

  /** Stops The Climbers */
  // public void stop() {
  //   leftSpeed = 0;
  //   rightSpeed = 0;
  //   rightMovingUp = false;
  //   leftMovingUp = false;
  //   //inMotion = false;
  // }

  /** gets the limits */
  // public boolean getLimits() {
  //   if(rightLimit.get() || leftLimit.get() ) {
  //     return true;
  //   }
  //   return false;
  // }



  /** If the arm reaches the bottom limit, it will stop Same for top */
  public void periodic() {

    if (Math.abs(leftSpeed) > 0.4) {
      leftSpeed = Math.signum(leftSpeed) * 0.4;
    }

    if (Math.abs(rightSpeed) > 0.4) {
      rightSpeed = Math.signum(rightSpeed) * 0.4;
    }

    if (leftLimit.get() && leftSpeed < 0) {
      leftMotor.set(0);
    } else {
      leftMotor.set(-leftSpeed);
    }

    if (rightLimit.get() && rightSpeed < 0) {
      rightMotor.set(0);
    } else {
      rightMotor.set(-rightSpeed);
    }
    // if (rightLimit.get() && !rightMovingUp && !rightAtTop) {
    //   rightAtBottom = true;
    // }
    // else if (leftLimit.get() && !leftMovingUp && !leftAtTop) {
    //   leftAtBottom = true;
    // } 
    // else if (!leftLimit.get()) {
    //   leftAtBottom = false;
    //   rightAtBottom= false;
    // }

    // if (rightLimit.get() && rightMovingUp && !rightAtBottom) { //maybe make rising edge detector instead of this
    //   rightAtTop = true;
    // }
    // else if (leftLimit.get() && leftMovingUp && !leftAtBottom) {
    //   leftAtTop = true;
    // }
    // else if (!rightLimit.get()) {
    //   leftAtTop = false;
    //   rightAtTop = false;
    // }

    // double setRight = 0;
    // double setLeft = 0;

    // if (!(rightAtBottom && !rightMovingUp)) {
    //   setRight = rightSpeed;
    // } 
    // if (!(rightAtTop && rightMovingUp)) {
    //   setRight = rightSpeed;
    // } 
    // if (!(leftAtBottom && !leftMovingUp)) {
    //   setLeft = leftSpeed;
    // } 
    // if (!(leftAtTop && leftMovingUp)) {
    //   setLeft = leftSpeed;
    // } 

    // leftMotor.set(setLeft);
    // rightMotor.set(setRight);


  }
}
