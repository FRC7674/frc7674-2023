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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Wrist extends SubsystemBase {

//Motor
public CANSparkMax wristRotate = new CANSparkMax(11, MotorType.kBrushless);
public CANSparkMax wristAngle = new CANSparkMax(10,MotorType.kBrushless);

public DigitalInput magneticWristCenterSwitch = new DigitalInput(0);

//PID
/*
    private SparkMaxPIDController pidController = wristRotate.getPIDController();
    public RelativeEncoder wristEncoder = wristRotate.getEncoder();
*/
    
      // dont know if this is needed or not \\
public DigitalInput wristInput = new DigitalInput(10);

//Creates a new Wrist.
    public Wrist() {

      

  }
    @Override
    public void periodic() {
    //This method will be called once per scheduler run
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
      if (magneticWristCenterSwitch.get() == true) 
      {
        return false;
      } 
      else 
      {
        return true;
      }
    }
}