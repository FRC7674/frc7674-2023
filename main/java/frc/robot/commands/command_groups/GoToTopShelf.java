// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Slide.SetArmSlidePosition;
import frc.robot.commands.Wrist.SetWristAnglePosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToTopShelf extends SequentialCommandGroup {
  /** Creates a new GoToTopShelf. */
  public GoToTopShelf() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.parallel(
        new SetArmPosition(64000),
        new SetWristAnglePosition(93)),
      new SetArmSlidePosition(58000)  
    );
  }
}
