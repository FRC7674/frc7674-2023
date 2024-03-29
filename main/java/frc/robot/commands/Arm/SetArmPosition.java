// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.utils.HelperFunctions;

public class SetArmPosition extends CommandBase {
 
  private double Position = 0.0;
  /** Creates a new SetArmPosition. */
  public SetArmPosition(double Position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.arm);
    this.Position = Position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.arm.armGoToPosition(Position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return HelperFunctions.deadband(Robot.arm.getArmPosition() - Position, 1000) == 0;
  }
}