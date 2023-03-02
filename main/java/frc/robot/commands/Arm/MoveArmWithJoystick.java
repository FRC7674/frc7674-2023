// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.HelperFunctions;

public class MoveArmWithJoystick extends CommandBase {
  /** Creates a new MoveArmWithJoystick. */

  private RobotContainer robotContainer = new RobotContainer();

  public MoveArmWithJoystick() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double armSlideValue = robotContainer.getOperatorLeftStick().getFirst(); //up is
    double safeArmSlideValue = HelperFunctions.deadband(armSlideValue, 0.25)* -0.5;
    Robot.arm.setSlideVoltage(safeArmSlideValue);

    double armMoveValue = robotContainer.getOperatorLeftStick().getSecond(); //up is
    double safeArmMoveValue = HelperFunctions.deadband(armMoveValue, 0.25)* -1; // was /0.5
    Robot.arm.setArmVoltage(safeArmMoveValue);

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
