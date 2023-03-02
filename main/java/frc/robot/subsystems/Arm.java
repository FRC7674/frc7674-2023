// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Arm.MoveArmWithJoystick;

public class Arm extends SubsystemBase {

  public TalonFX armSlide = new TalonFX(7);
  public TalonFX armAngleLead = new TalonFX(8);
  public TalonFX armAngleFollow = new TalonFX(9);


  /** Creates a new Arm. */
  public Arm() {

    armAngleFollow.follow(armAngleLead);
    armSlide.setInverted(true);

    armSlide.setNeutralMode(NeutralMode.Coast);
    armAngleLead.setNeutralMode(NeutralMode.Coast);
    armAngleFollow.setNeutralMode(NeutralMode.Coast);

    armAngleLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,30);

    armAngleLead.setSensorPhase(false); //for positive + positive
  
   armAngleLead.configNominalOutputForward(0,30);
   armAngleLead.configNominalOutputReverse(0,30);
   armAngleLead.configPeakOutputForward(0.25,30);
   armAngleLead.configPeakOutputReverse(-0.25,30);

    armAngleLead.configAllowableClosedloopError(0, 100, 30);

   armAngleLead.config_kF(0, 0,30); // was 0.1150875 0.0639375 // change to 0 and now the motors stop correctly
   armAngleLead.config_kI(0, 0,30); 
   armAngleLead.config_kP(0, .08,30); // 0.0000006394 // tried bigger number and it worked // change to make speed bigger and not f
   armAngleLead.config_kD(0, 0.8,30);
//-------------------------------------------------------------------------------------------------------------------------------------\\
 
                 // Arm Slide \\

  armSlide.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,30);

  armSlide.setSensorPhase(false); //for positive + positive

  armSlide.configNominalOutputForward(0,30);
  armSlide.configNominalOutputReverse(0,30);
  armSlide.configPeakOutputForward(0.1,30);
  armSlide.configPeakOutputReverse(-0.1,30);

  armSlide.configAllowableClosedloopError(0, 100, 30);

  armSlide.config_kF(0, 0,30);
  armSlide.config_kI(0, 0,30); 
  armSlide.config_kP(0, 0,30); 
  armSlide.config_kD(0, 0,30);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getCurrentPosition() {
    return 0;
  }

  public void setArmVoltage(double voltage){
    armAngleLead.set(ControlMode.PercentOutput, voltage);
  }
  public void setSlideVoltage(double voltage){
    armSlide.set(ControlMode.PercentOutput, voltage);
  }

  public double getArmPosition(){
    return armAngleLead.getSelectedSensorPosition();
  }

  public double getSlidePosition(){
    return armSlide.getSelectedSensorPosition();
  }

  public void armGoToPosition(double Position){
    armAngleLead.set(TalonFXControlMode.Position, Position);
  }

  public void armSlideGoToPosition(double Position){
    armSlide.set(TalonFXControlMode.Position, Position);
  }

  public double getArmVelocity(){
    return armAngleLead.getSelectedSensorVelocity();
  }

  public double getArmSlideVelocity(){
    return armSlide.getSelectedSensorVelocity();
  }

      // encoders \\
  public void zeroArmEncoders(){
    armAngleLead.setSelectedSensorPosition(0);
    armAngleFollow.setSelectedSensorPosition(0);
  }
      // encoders \\
  public void setArmEncoders(double Position){
    armAngleLead.setSelectedSensorPosition(Position);
    armAngleFollow.setSelectedSensorPosition(Position);
  }

      // encoders \\
  public void zeroArmSlideEncoders(){
    armSlide.setSelectedSensorPosition(0);
  }

      // encoders \\
  public void setSlideEncoders(double Position){
    armSlide.setSelectedSensorPosition(Position);
  }
}
