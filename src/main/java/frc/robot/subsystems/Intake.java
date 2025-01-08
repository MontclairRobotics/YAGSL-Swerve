package frc.robot.subsystems;



import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;


public class Intake extends SubsystemBase {

  private final SparkMax topMotor =
      new SparkMax(Ports.INTAKE_TOP_MOTOR, MotorType.kBrushless);
  private final SparkMax bottomMotor =
      new SparkMax(Ports.INTAKE_BOTTOM_MOTOR, MotorType.kBrushless);
  
  
 

  public Intake() {

    SparkMaxConfig config = new SparkMaxConfig();
    config
      .inverted(true)
      .idleMode(IdleMode.kCoast);
    
    topMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


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

  }

  public void teleopInit() {
    stop();
  }
}
