// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.HelperFunctions;
import frc.robot.subsystems.Arm;

public class MoveArmWithJoystick extends CommandBase {
  /** Creates a new MoveArmWithJoystick. */
  private RobotContainer robotContainer = new RobotContainer();

  public MoveArmWithJoystick() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Arm.armAnglePosition = Robot.arm.getArmPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

                            // Arm Move \\
    double armMoveValue = robotContainer.getOperatorLeftStick().getSecond(); //up is up down is down
    double safeArmMoveValue = HelperFunctions.deadband(armMoveValue, 0.1)* -1; // was 0.5 
    Arm.armAnglePosition = Arm.armAnglePosition + safeArmMoveValue * 400; //updating armAnglePosition with speed+joystick control
    Robot.arm.armGoToPosition(Arm.armAnglePosition);

     //move soft limit high
     if (Arm.armAnglePosition >= 85000) {
      Arm.armAnglePosition = 85000;
      Robot.arm.armGoToPosition(Arm.armAnglePosition);
    }
    //move soft limit low
    if (Arm.armAnglePosition <= 0) {
      Arm.armAnglePosition = 0;
      Robot.arm.armGoToPosition(Arm.armAnglePosition);
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
