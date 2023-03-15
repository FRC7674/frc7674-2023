// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Arm.SetArmSlidePosition;
import frc.robot.commands.Wrist.GripperToggle;
import frc.robot.commands.Wrist.SetWristAnglePosition;
import frc.robot.commands.drivetrain.SetDrivePO;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAndMove extends SequentialCommandGroup {
  /** Creates a new ScoreAndMove. */
  public ScoreAndMove() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    Commands.sequence( 
      new GripperToggle()//,
      //Commands.parallel(
        //new SetArmPosition(75000),// move arm to angle 
        //new SetWristAnglePosition(75)//,
        //new SetArmSlidePosition(65000)
      //),
      
      //new GripperToggle(),
      //new WaitCommand(1),
      //Commands.deadline(new WaitCommand(2), new SetDrivePO(0.1))
    );
  }
}
