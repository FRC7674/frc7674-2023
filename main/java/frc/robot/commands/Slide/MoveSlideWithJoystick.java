// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Slide;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.HelperFunctions;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Slide;

public class MoveSlideWithJoystick extends CommandBase {
  /** Creates a new MoveArmWithJoystick. */
  private RobotContainer robotContainer = new RobotContainer();

  public MoveSlideWithJoystick() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.slide);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Slide.armSlidePosition = Robot.slide.getSlidePosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Arm Slide \\ 
    double armSlideValue = robotContainer.getOperatorLeftStick().getFirst(); //left is up
    double safeArmSlideValue = HelperFunctions.deadband(armSlideValue, 0.1)* -0.5; // changed deadband
    Slide.armSlidePosition = Slide.armSlidePosition + safeArmSlideValue * 700; //updating armSlidePosition with speed+joystick control
    Robot.slide.armSlideGoToPosition(Slide.armSlidePosition);

    //slide soft limit high
    if (Slide.armSlidePosition >= 75000) {
      Slide.armSlidePosition = 75000;
      Robot.slide.armSlideGoToPosition(Slide.armSlidePosition);
    }
    //slide soft limit low
    if (Slide.armSlidePosition <= 0) {
      Slide.armSlidePosition = 0;
      Robot.slide.armSlideGoToPosition(Slide.armSlidePosition);
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
