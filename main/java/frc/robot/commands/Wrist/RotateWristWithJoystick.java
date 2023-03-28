// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.HelperFunctions;

public class RotateWristWithJoystick extends CommandBase {
  /** Creates a new RotateWristWithJoystick. */

  private RobotContainer robotContainer = new RobotContainer();

  public double wristRotatePosition;
  public double wristAnglePosition;

  public RotateWristWithJoystick() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristRotatePosition = Robot.wrist.getWristRotatePosition();
    wristAnglePosition = Robot.wrist.getWristPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()  {
  
                        // Wrist Rotate \\
    double moveValue = robotContainer.getOperatorRightStick().getFirst(); //get x axis (no deadband)
    double safeMoveValue = HelperFunctions.deadband(moveValue, 0.1)* 1; //adding in deadband (use this) // changed deadband
    //Robot.wrist.setWristRotateVoltage(safeMoveValue);
    wristRotatePosition = wristRotatePosition + safeMoveValue * 1.1; //updating wristRotatePosition with speed+joystick control
    Robot.wrist.setRotatePosition(wristRotatePosition); //setting wrist rotate position

    Robot.wrist.getWristRotateError = wristRotatePosition - Robot.wrist.getWristRotatePosition();

    //rotate soft limit high
    if (wristRotatePosition >= 150) {
      wristRotatePosition = 150;
      Robot.wrist.setRotatePosition(wristRotatePosition);
    }
    //rotate soft limit low
    if (wristRotatePosition <= -150) {
      wristRotatePosition = -150;
      Robot.wrist.setRotatePosition(wristRotatePosition);
    }

                    // Wrist Angle \\
    double pivotValue = robotContainer.getOperatorRightStick().getSecond(); //get y axis (no deadband)
    double safePivotValue = HelperFunctions.deadband(pivotValue, 0.1)* 0.5; //adding in deadband (use this) 
    // Robot.wrist.setWristVoltage(safePivotValue);
    wristAnglePosition = wristAnglePosition + safePivotValue * 3; //updating wristAnglePosition with speed+joystick control
    Robot.wrist.setAnglePosition(wristAnglePosition); //setting wrist angle position

    Robot.wrist.getWristAngleError = wristAnglePosition - Robot.wrist.getWristPosition();
    
    //angle soft limit high
    if (wristAnglePosition >= 115) {
      wristAnglePosition = 115;
      Robot.wrist.setAnglePosition(wristAnglePosition);
    }
    //angle soft limit low
    if (wristAnglePosition <= 0) {
      wristAnglePosition = 0;
      Robot.wrist.setAnglePosition(wristAnglePosition);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
