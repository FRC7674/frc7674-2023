// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Wrist.RotateWristWithJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Wrist extends SubsystemBase {

//Motor
public CANSparkMax wristRotate = new CANSparkMax(11, MotorType.kBrushless);
public CANSparkMax wristAngle = new CANSparkMax(10, MotorType.kBrushless);

public DigitalInput magneticWristCenterSwitch = new DigitalInput(0);

public RelativeEncoder rotateEncoder = wristRotate.getEncoder();
public SparkMaxPIDController rotatePidController = wristRotate.getPIDController();
public RelativeEncoder angleEncoder = wristAngle.getEncoder();
public SparkMaxPIDController anglePidController = wristAngle.getPIDController();

public double getWristRotateError;
public double getWristAngleError;

public double wristRotatePosition;
public double wristAnglePosition;

//Creates a new Wrist.
    public Wrist() {

     // wristRotate.setInverted(true); // to make left joystick left spin

      rotatePidController.setFF(0.00008510);
      rotatePidController.setP(0.065);

      anglePidController.setFF(0.00009091);
      anglePidController.setP(0.15);
    }
    
    @Override
    public void periodic() {
    //This method will be called once per scheduler run
    }

    public void initDefaultCommand(){
      setDefaultCommand(new RotateWristWithJoystick());
    }

    public static double getDigitalInput() {
      return 0;
    }

    public void setWristVoltage(double voltage){
      wristAngle.set(voltage);
    }

    public void setWristRotateVoltage(double voltage){
      wristRotate.set(voltage);
    }

    public boolean getWristSwitchState(){
      if (magneticWristCenterSwitch.get() == true) {
        return false;
      } 
      else {
        return true;
      }
    }

public double getWristPosition(){
  return angleEncoder.getPosition();
}

public double getWristRotatePosition(){
  return rotateEncoder.getPosition();
}

public void setRotatePosition(double Position){
  rotatePidController.setReference(Position,CANSparkMax.ControlType.kPosition);
}

public double getWristRotateVelocity(){
  return rotateEncoder.getVelocity();
}

public void setAnglePosition(double Position){
  anglePidController.setReference(Position, CANSparkMax.ControlType.kPosition);
}

public double getWristAngleVelocity(){
  return angleEncoder.getVelocity();
}

}