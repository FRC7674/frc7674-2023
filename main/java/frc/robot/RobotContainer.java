// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriverTest;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Arm.SetArmVoltage;
import frc.robot.commands.Arm.SetSlideVoltage;
import frc.robot.commands.Arm.setArmEncoderPos;
import frc.robot.commands.Arm.setArmSlideEncoderPos;
import frc.robot.commands.Wrist.GripperToggle;
import frc.robot.commands.Wrist.RotateToSwitch;
import frc.robot.commands.Wrist.SetWristAnglePosition;
import frc.robot.commands.Wrist.SetWristRotatePosition;
import frc.robot.commands.Wrist.SetWristRotateVoltage;
import frc.robot.commands.Wrist.SetWristVoltage;
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
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = 
    new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final CommandXboxController m_operatorController = 
    new CommandXboxController(OperatorConstants.kOperatorControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
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
 
                         // WRIST ROTATE \\
    m_operatorController.x().whileTrue(new SetWristRotatePosition(0));
    m_operatorController.b().whileTrue(new SetWristRotatePosition(500));
   
                         // WRIST MOVE \\
    m_operatorController.a().whileTrue(new SetWristAnglePosition(0)); 
    m_operatorController.y().whileTrue(new SetWristAnglePosition(124));

    m_operatorController.rightBumper().whileTrue(new RotateToSwitch());

    m_operatorController.leftBumper().whileTrue(new GripperToggle());

                         // ARM MOVE \\
    //m_operatorController.leftTrigger().whileTrue(new SetArmVoltage(0.5)); //goes out
    //m_operatorController.rightTrigger().whileTrue(new SetArmVoltage(-0.5)); //goes in

   // m_operatorController.povLeft().whileTrue(new setArmEncoderPos());
    m_operatorController.rightTrigger().whileTrue(new SetArmPosition(2500));
    m_operatorController.leftTrigger().whileTrue(new SetArmPosition(71000)); //other


                         // ARM SLIDE \\
    m_operatorController.povLeft().whileTrue(new setArmSlideEncoderPos());

    m_operatorController.povUp().whileTrue(new SetSlideVoltage(0.1));
    m_operatorController.povDown().whileTrue(new SetSlideVoltage(-0.1));


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
    double leftY = -1.0 * m_driverController.getLeftY();
    return new Pair<>(leftX, leftY);
  }

  public Pair<Double, Double> getRightStick() {
    double rightX = m_driverController.getRightX();
    double rightY = -1.0 * m_driverController.getRightY();
    return new Pair<>(rightX, rightY);
  }

  public Pair<Double, Double> getOperatorLeftStick() {
    double leftX = m_operatorController.getLeftX();
    double leftY = -1.0 * m_operatorController.getLeftY();
    return new Pair<>(leftX, leftY);
  }

  public Pair<Double, Double> getOperatorRightStick() {
    double rightX = m_operatorController.getRightX();
    double rightY = -1.0 * m_operatorController.getRightY();
    return new Pair<>(rightX, rightY);
  }
}
