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
  
  public TalonFX armSlide = new TalonFX(7);
  public TalonFX armAngleLead = new TalonFX(8);
  public TalonFX armAngleFollow = new TalonFX(9);

  public static double armAnglePosition;
  public static double armSlidePosition;

  public static double currentArmAnglePosition;

  //int armSlideRevLimitSwitchTripped = Robot.arm.armSlide.getSensorCollection().isRevLimitSwitchClosed();
  //int armAngleRevLimitSwitchTripped = Robot.arm.armAngleLead.getSensorCollection().isRevLimitSwitchClosed();

  /** Creates a new Arm. */
  public Arm() {

    armAngleFollow.follow(armAngleLead);
    armSlide.setInverted(true);

    armSlide.setNeutralMode(NeutralMode.Brake);
    armAngleLead.setNeutralMode(NeutralMode.Brake);
    armAngleFollow.setNeutralMode(NeutralMode.Brake);

    armAngleLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,30);

    armAngleLead.setSensorPhase(false); //for positive + positive
  
   armAngleLead.configNominalOutputForward(0,30);
   armAngleLead.configNominalOutputReverse(0,30);
   armAngleLead.configPeakOutputForward(0.25,30);
   armAngleLead.configPeakOutputReverse(-0.25,30);

    armAngleLead.configAllowableClosedloopError(0, 100, 30);

   armAngleLead.config_kF(0, 0,30); // change to 0 and now the motors stop correctly
   armAngleLead.config_kI(0, 0,30); 
   armAngleLead.config_kP(0, .08,30); // tried bigger number and it worked // change to make speed bigger and not f
   armAngleLead.config_kD(0, 0.8,30);

//-------------------------------------------------------------------------------------------------------------------------------------\\
 
                 // Arm Slide \\

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
    /*
    //zero encoders with limit switches
    if (armAngleRevLimitSwitchTripped == 1) {
      armAngleLead.configClearPositionOnLimitR(true, 30);
      armAngleFollow.configClearPositionOnLimitR(true, 30);
    }
    */
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
  //--------------------------------------------------------\\
                  // SLIDE \\

      // encoders \\
  public void zeroArmSlideEncoders(){
    armSlide.setSelectedSensorPosition(0);
  }

      // encoders \\
  public void setSlideEncoders(double Position){
    armSlide.setSelectedSensorPosition(Position);
    /*
    //zero encoders with limit switches
    if (armSlideRevLimitSwitchTripped == 1) {
      armSlide.configClearPositionOnLimitR(true, 30);
    }
    */
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