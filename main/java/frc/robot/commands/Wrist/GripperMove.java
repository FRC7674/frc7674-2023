// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GripperMove extends InstantCommand {
  public GripperMove() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public boolean m_updown;
  
  public GripperMove(boolean state){
    addRequirements(Robot.pneumatics);

    m_updown = state;
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (m_updown == true) 
    {
      Robot.pneumatics.gripperSolenoid1.set(Value.kForward); 
    } 
    else 
    {
      Robot.pneumatics.gripperSolenoid1.set(Value.kReverse); 
    }

  }
}
