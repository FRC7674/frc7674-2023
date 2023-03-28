// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Slide extends SubsystemBase {

  public TalonFX armSlide = new TalonFX(7);
  public static double armSlidePosition;
  
  /** Creates a new Slide. */
  public Slide() {
 
                 // Arm Slide \\

  armSlide.setInverted(true);
  armSlide.setNeutralMode(NeutralMode.Brake);
  armSlide.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,30);

  armSlide.setSensorPhase(false); //for positive + positive

  armSlide.configNominalOutputForward(0,30);
  armSlide.configNominalOutputReverse(0,30);
  armSlide.configPeakOutputForward(0.6,30); // up
  armSlide.configPeakOutputReverse(-0.3,30); // down

  armSlide.configAllowableClosedloopError(0, 100, 30);

  armSlide.config_kF(0, 0,30);
  armSlide.config_kI(0, 0.1,30); 
  armSlide.config_kP(0, 0.35,30); 
  armSlide.config_kD(0, 0,30);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

                  // SLIDE \\

      // encoders \\
      public void zeroArmSlideEncoders(){
        armSlide.setSelectedSensorPosition(0);
      }
    
          // encoders \\
      public void setSlideEncoders(double Position){
      } 
      
      public double getArmSlideVelocity(){
        return armSlide.getSelectedSensorVelocity();
      }
      
      public void armSlideGoToPosition(double Position){
        armSlide.set(TalonFXControlMode.Position, Position);
      }
      
      public double getSlidePosition(){
        return armSlide.getSelectedSensorPosition();
      }
      
      public void setSlideVoltage(double voltage){
        armSlide.set(ControlMode.PercentOutput, voltage);
      }
}
