// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Slide.SetArmSlidePosition;
import frc.robot.commands.Wrist.AutoToggleGripper;
import frc.robot.commands.Wrist.GripperToggle;
import frc.robot.commands.Wrist.SetWristAnglePosition;
import frc.robot.commands.drivetrain.DriveDistance;
import frc.robot.commands.drivetrain.SetDrivePO;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAndMove extends SequentialCommandGroup {
  /** Creates a new ScoreAndMove. */
  public ScoreAndMove() {
    // Add your commands in the addCommands() call, e.g.
     addCommands(

   Commands.parallel(
     new SetArmPosition(47000),
     new SetWristAnglePosition(58),
     new SetArmSlidePosition(0)
   ),
   new WaitCommand(.5),
   Commands.parallel(
    new SetArmSlidePosition(58000),
    new SetArmPosition(47000),
    new SetWristAnglePosition(64)
  ),
    new WaitCommand(.5),
  Commands.parallel(
    new SetArmSlidePosition(58000),
    new SetArmPosition(56000),
    new SetWristAnglePosition(64)
  ),
   new WaitCommand(1),
   new AutoToggleGripper(),
   new WaitCommand(1),
    Commands.parallel(
     new SetArmPosition(0),
     new SetWristAnglePosition(0),
     new SetArmSlidePosition(0)
   ),
   new WaitCommand(1),
   new DriveDistance(-5.25, 0.5)
    );
  }
}
