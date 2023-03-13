// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Arm.SetArmSlidePosition;
import frc.robot.commands.Wrist.GripperToggle;
import frc.robot.commands.Wrist.SetWristAnglePosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto extends SequentialCommandGroup {
  /** Creates a new Auto. */
  public Auto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // position the slide, arm, wrist to place cone on top peg
    // release the cone - gripper toggle
    // put the slide, arm, wrist in starting config
    // back out of community area - drive distance

  }
}
