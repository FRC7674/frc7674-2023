// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.utils.DriveSignal;

public class SetDrivePO extends CommandBase {
  double percentage = 0.0;
  /** Creates a new SetDrivePO. */
  public SetDrivePO(double percentageM) {
    this.percentage = percentageM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSignal signal = new DriveSignal(percentage, percentage);
    Robot.drivetrain.drive(ControlMode.PercentOutput, signal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSignal signal = new DriveSignal(0, 0);
    Robot.drivetrain.drive(ControlMode.PercentOutput, signal);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
