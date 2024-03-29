// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class CameraControl extends CommandBase {

  double d = 0.0;
  /** Creates a new CameraControl.*/
  public CameraControl(double d) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.d = d;
    addRequirements(Robot.limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.limelight.setServo(d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
