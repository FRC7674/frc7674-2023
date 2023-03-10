// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Arm.SetArmSlidePosition;
import frc.robot.commands.Arm.SetArmVoltage;
import frc.robot.commands.Arm.SetSlideVoltage;
import frc.robot.commands.Arm.setArmEncoderPos;
import frc.robot.commands.Arm.setArmSlideEncoderPos;
import frc.robot.commands.Presets.GroundLevel;
import frc.robot.commands.Wrist.GripperToggle;
import frc.robot.commands.Wrist.RotateToSwitch;
import frc.robot.commands.Wrist.SetWristAnglePosition;
import frc.robot.commands.Wrist.SetWristRotatePosition;
import frc.robot.commands.Wrist.SetWristRotateVoltage;
import frc.robot.commands.Wrist.SetWristVoltage;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * genius
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //add classes
  GroundLevel groundLevel = new GroundLevel();
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed

    public final CommandXboxController m_driverController = 
    new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public final CommandXboxController m_operatorController = 
    new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /*
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

                           // Starting Config \\ 
     m_operatorController.x().whileTrue(new SetArmPosition(0)); 
     m_operatorController.x().whileTrue(new SetWristAnglePosition(0)); 
     m_operatorController.x().whileTrue(new SetArmSlidePosition(0)); 

                             // High Level \\
    m_operatorController.y().whileTrue(new SetArmPosition(56000)); 
    m_operatorController.y().whileTrue(new SetWristAnglePosition(82)); 
    m_operatorController.y().whileTrue(new SetArmSlidePosition(58000)); 

                            // Medium Level \\
    m_operatorController.b().whileTrue(new SetArmPosition(57000)); 
    m_operatorController.b().whileTrue(new SetWristAnglePosition(78)); 
    m_operatorController.b().whileTrue(new SetArmSlidePosition(0)); 

                             // Ground Level \\ 
     m_operatorController.a().whileTrue(new SetArmPosition(83000)); 
     m_operatorController.a().whileTrue(new SetWristAnglePosition(104)); 
     m_operatorController.a().whileTrue(new SetArmSlidePosition(0)); 

                            // Top Shelf \\ 
    m_operatorController.povUp().whileTrue(new SetArmPosition(64000)); 
    m_operatorController.povUp().whileTrue(new SetWristAnglePosition(93)); 
    m_operatorController.povUp().whileTrue(new SetArmSlidePosition(58000)); 

                             // Shelf Level \\ 
    m_operatorController.povDown().whileTrue(new SetArmPosition(60000)); 
    m_operatorController.povDown().whileTrue(new SetWristAnglePosition(76)); 
    m_operatorController.povDown().whileTrue(new SetArmSlidePosition(0)); 

                            // Wrist Rotate \\ 
    m_operatorController.povLeft().whileTrue(new SetWristRotatePosition(0));
    m_operatorController.povRight().whileTrue(new SetWristRotatePosition(75));
   
                             // Gripper \\ 
    m_operatorController.leftBumper().whileTrue(new GripperToggle());

  }

  /*
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
   // // An example command will be run in autonomous
   // return new Command();
  //}
               // Driver \\
  public Pair<Double, Double> getLeftStick() {
    double leftX = m_driverController.getLeftX();
    if (leftX < 0) {
      leftX = -(leftX * leftX);
  } else {
      leftX = leftX * leftX;
  }
    double leftY = -1.0 * m_driverController.getLeftY();
    if (leftY < 0) {
      leftY = -(leftY * leftY);
  } else {
      leftY = leftY * leftY;
  }
    return new Pair<>(leftX, leftY);
  }

  public Pair<Double, Double> getRightStick() {
    double rightX = m_driverController.getRightX();
    if (rightX < 0) {
      rightX = -(rightX * rightX);
  } else {
      rightX = rightX * rightX;
  }
    double rightY = -1.0 * m_driverController.getRightY();
    if (rightY < 0) {
      rightY = -(rightY * rightY);
  } else {
      rightY = rightY * rightY;
  }
    return new Pair<>(rightX, rightY);
  }
  
               // Operator \\
  public Pair<Double, Double> getOperatorLeftStick() {
    double leftX = m_operatorController.getLeftX();
    if (leftX < 0) {
      leftX = -(leftX * leftX);
  } else {
      leftX = leftX * leftX;
  }
    double leftY = -1.0 * m_operatorController.getLeftY();
    if (leftY < 0) {
      leftY = -(leftY * leftY);
  } else {
      leftY = leftY * leftY;
  }
    return new Pair<>(leftX, leftY);
}

  public Pair<Double, Double> getOperatorRightStick() {
    double rightX = m_operatorController.getRightX();
    if (rightX < 0) {
      rightX = -(rightX * rightX);
  } else {
      rightX = rightX * rightX;
  }
    double rightY = -1.0 * m_operatorController.getRightY();
    if (rightY < 0) {
      rightY = -(rightY * rightY);
  } else {
      rightY = rightY * rightY;
  }
    return new Pair<>(rightX, rightY);
  }
}