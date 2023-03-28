// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Arm.MoveArmWithJoystick;

public class Arm extends SubsystemBase {
  
  
  public TalonFX armAngleLead = new TalonFX(8);
  public TalonFX armAngleFollow = new TalonFX(9);

  public static double armAnglePosition;
  public static double currentArmAnglePosition;

  /** Creates a new Arm. */
  public Arm() {

    armAngleFollow.follow(armAngleLead);

    armAngleLead.setNeutralMode(NeutralMode.Brake);
    armAngleFollow.setNeutralMode(NeutralMode.Brake);

    armAngleLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,30);

    armAngleLead.setSensorPhase(false); //for positive + positive
  
    armAngleLead.configNominalOutputForward(0,30);
    armAngleLead.configNominalOutputReverse(0,30);
    armAngleLead.configPeakOutputForward(0.2,30); 
    armAngleLead.configPeakOutputReverse(-0.2,30); 

    armAngleLead.configAllowableClosedloopError(0, 100, 30);

    armAngleLead.config_kF(0, 0,30); // change to 0 and now the motors stop correctly
    armAngleLead.config_kI(0, 0,30); 
    armAngleLead.config_kP(0, .08,30); // tried bigger number and it worked // change to make speed bigger and not f
    armAngleLead.config_kD(0, 0.8,30);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getCurrentPosition() {
    return 0;
  }

                  // ARM \\

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

  public double getArmVelocity(){
    return armAngleLead.getSelectedSensorVelocity();
  }

  public void armGoToPosition(double Position){
    armAngleLead.set(TalonFXControlMode.Position, Position);
  }

  public double getArmPosition(){
    return armAngleLead.getSelectedSensorPosition();
  }
  
  public void setArmVoltage(double voltage){
    armAngleLead.set(ControlMode.PercentOutput, voltage);
  }
 
}