// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  public TalonFX armSlide = new TalonFX(7);
  public TalonFX armAngleLead = new TalonFX(8);
  public TalonFX armAngleFollow = new TalonFX(9);

  // Getting PID Controller and Encoder for elevator
 // public RelativeEncoder elevatorEncoder = armSlide.getEncoder();

  /** Creates a new Arm. */
  public Arm() {

    armAngleFollow.follow(armAngleLead);
    armSlide.setInverted(true);


    armSlide.setNeutralMode(NeutralMode.Brake);
    armAngleLead.setNeutralMode(NeutralMode.Brake);
    armAngleFollow.setNeutralMode(NeutralMode.Brake);

    // Arm Angle Follow Motor PID Setup
    this.armAngleFollow.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    this.armAngleFollow.configClosedloopRamp(0.25);
   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
/*  (code stollen from 319)
  private void pidUp(){
    pidController.setP(Constants.ArmSlideConstants.PID.kPUp);
    pidController.setI(Constants.ArmSlideConstants.PID.kIUp);
    pidController.setD(Constants.ArmSlideConstants.PID.kDUp);
    pidController.setFF(Constants.ArmSlideConstants.PID.fGainUp);
  }

  private void pidDown(){
    pidController.setP(Constants.ArmSlideConstants.PID.kPDown);
    pidController.setI(Constants.ArmSlideConstants.PID.kIDown);
    pidController.setD(Constants.ArmSlideConstants.PID.kDDown);
    pidController.setFF(Constants.ArmSlideConstants.PID.fGainDown);
  }
*/
/* 
public double getCurrentPosition() {
  return this.armSlide.getPosition();
}
*/

  public double getCurrentPosition() {
    return 0;
  }

  public void setArmVoltage(double voltage){
    armAngleLead.set(ControlMode.PercentOutput, voltage);
  }
  public void setSlideVoltage(double voltage){
    armSlide.set(ControlMode.PercentOutput, voltage);
  }
}
