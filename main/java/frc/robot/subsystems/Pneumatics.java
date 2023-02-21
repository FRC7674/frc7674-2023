// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {


  public Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  public boolean iseEnabled;
  public boolean getPressureSwitchValue = compressor.getPressureSwitchValue();
  private double current = compressor.getCurrent();

  public DoubleSolenoid gripperSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  public DoubleSolenoid gripperSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  /** Creates a new Pneumatics. */
  public Pneumatics() {
  
    compressor.enableDigital();
 //   isEnabled = compressor.isEnabled();
    getPressureSwitchValue = compressor.getPressureSwitchValue();
    current = compressor.getCurrent();


    gripperSolenoid1.set(Value.kForward);
    gripperSolenoid2.set(Value.kForward);
  

  }

  public void toggleGripper1Solenoid(){
    gripperSolenoid1.toggle();
  }

  public void toggleGripper2Solenoid(){
    gripperSolenoid2.toggle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
