// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveDistance extends CommandBase {

double distance = 0.0;
double percentOutput = 0;
Boolean leftFinished = false;
Boolean rightFinished = false;

  /** Creates a new DriveDistance. */
  public DriveDistance(double distance_ , double percentOutput_ ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.drivetrain);
    distance = distance_;
    percentOutput = percentOutput_;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.drivetrain.setDrivetrainPositionToZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.drivetrain.drive(ControlMode.PercentOutput, percentOutput, percentOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.drivetrain.drive(ControlMode.PercentOutput, 0, 0);
    Robot.drivetrain.setMotorNeutralModes(IdleMode.kBrake); //no input needed, function sets them all to brake
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if((distance > 0 && percentOutput < 0) || (distance < 0 && percentOutput > 0)) {
      percentOutput = - percentOutput;
    }


    if (distance > 0) {
      leftFinished = Robot.drivetrain.getLeftLeadDriveDistanceMeters() >= distance;
    } else {
      leftFinished = Robot.drivetrain.getLeftLeadDriveDistanceMeters() <= distance;
    }


    if (distance > 0) {
      rightFinished = Robot.drivetrain.getRightLeadDriveDistanceMeters() >= distance;
    } else {
      rightFinished = Robot.drivetrain.getRightLeadDriveDistanceMeters() <= distance;
    }


    if (leftFinished && rightFinished) {
      return true;
    } else {
      return false;
    }
  }
}