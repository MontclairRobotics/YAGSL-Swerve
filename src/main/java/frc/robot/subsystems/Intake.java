package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;


public class Intake extends SubsystemBase {

  private final CANSparkMax topMotor =
      new CANSparkMax(Ports.INTAKE_TOP_MOTOR, MotorType.kBrushless);
  private final CANSparkMax bottomMotor =
      new CANSparkMax(Ports.INTAKE_BOTTOM_MOTOR, MotorType.kBrushless);
  
  
 

  public Intake() {
    topMotor.setInverted(true);
    bottomMotor.setInverted(true);
   
  }

  /** Accelerates motors to intake something */
  public void in() {
    topMotor.set(IntakeConstants.INTAKE_SPEED);
    bottomMotor.set(IntakeConstants.INTAKE_SPEED);
  }

  /** Reverse intake if gamepiece gets stuck */
  public void out() {
    topMotor.set(-IntakeConstants.INTAKE_SPEED);
    bottomMotor.set(-IntakeConstants.INTAKE_SPEED);
  }

  /** Stop intaking */
  public void stop() {
    topMotor.set(0);
    bottomMotor.set(0);
  }

  

  
  @Override
  public void periodic() {
    // if (getSensor()) {
    //   hasPickedUpNote = true;
    //   timeSinceNote.reset();
    //   timeSinceNote.start();
    // }

    // if (timeSinceNote.get() > 5) {
    //   hasPickedUpNote = false;
    //   timeSinceNote.stop();
    // }
  }

  public void teleopInit() {
    stop();
  }
}
