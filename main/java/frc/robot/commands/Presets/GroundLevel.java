// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Slide;

public class GroundLevel extends CommandBase {
  /** Creates a new GroundLevel. */

  public void groundLevel() {
    // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(Robot.arm);
      addRequirements(Robot.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm.armAnglePosition = 75000;
    Slide.armSlidePosition = 2000;
  
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
