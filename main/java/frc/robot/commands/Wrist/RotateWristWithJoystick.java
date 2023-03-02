// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.HelperFunctions;

public class RotateWristWithJoystick extends CommandBase {
  /** Creates a new RotateWristWithJoystick. */

  private RobotContainer robotContainer = new RobotContainer();

  public RotateWristWithJoystick() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()  {
  
    double moveValue = robotContainer.getOperatorRightStick().getFirst(); //get x axis
    double safeMoveValue = HelperFunctions.deadband(moveValue, 0.25)* 1;
    Robot.wrist.setWristRotateVoltage(safeMoveValue);

    double pivotValue = robotContainer.getOperatorRightStick().getSecond(); 
    double safePivotValue = HelperFunctions.deadband(pivotValue, 0.25)* 0.5;
    Robot.wrist.setWristVoltage(safePivotValue);
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
