// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Slide;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.utils.HelperFunctions;

public class SetArmSlidePosition extends CommandBase {

  private double Position = 0.0;
  /** Creates a new SetArmSlidePosition. */
  public SetArmSlidePosition(double Position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.slide);
    this.Position = Position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.slide.armSlideGoToPosition(Position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return HelperFunctions.deadband(Robot.slide.getSlidePosition() - Position, 800) == 0;
  }
}