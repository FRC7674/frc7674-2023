// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Arm.SetArmSlidePosition;
import frc.robot.commands.Robot.WaitForTime;
import frc.robot.commands.Wrist.SetWristAnglePosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Tester extends ParallelDeadlineGroup {
  /** Creates a new Tester. */
  public Tester() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new WaitForTime(.3)); // try wait for time? cause then that would actually finish right? just got to get the amount of time that we want
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetArmPosition(30000), new SetArmSlidePosition(0), new SetWristAnglePosition(50));
  }
}
