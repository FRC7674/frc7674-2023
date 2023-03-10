// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.security.Key;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.BobDrive;
import frc.robot.commands.Arm.MoveArmWithJoystick;
import frc.robot.commands.Arm.SetArmPosition;
import frc.robot.commands.Wrist.RotateWristWithJoystick;
import frc.robot.subsystems.Arm;
//import frc.robot.commands.autos.TestPath;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Wrist;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static Trajectory autoTrajectory = new Trajectory();
  public static Drivetrain drivetrain = new Drivetrain();
  public static Pneumatics pneumatics = new Pneumatics(); 
  public static Limelight limelight = new Limelight();
  public static Arm arm = new Arm();
  public static Wrist wrist = new Wrist();

  private Command m_teleopCommand = new BobDrive();

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    String trajectoryJSON = "PathWeaver//output//2metersforward.wpilib.json";
    try {
      Path testPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      autoTrajectory = TrajectoryUtil.fromPathweaverJson(testPath);
    }
    catch (IOException ex) {
      DriverStation.reportError("Unable to open Trajectory", ex.getStackTrace());
    } 
  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
  
    /* SmartDashboard.putNumber("Left Distance", drivetrain.getLeftLeadDriveDistanceMeters());
    SmartDashboard.putNumber("Right Distance", drivetrain.getRightLeadDriveDistanceMeters());
    SmartDashboard.putNumber("Left Ticks", drivetrain.getLeftLeadDriveDistanceTicks());
    SmartDashboard.putNumber("Right Ticks", drivetrain.getRightLeadDriveDistanceTicks());
    SmartDashboard.putNumber("Pose X", drivetrain.getPose().getX());
    SmartDashboard.putNumber("Pose Y", drivetrain.getPose().getY());
    SmartDashboard.putNumber("Fused Heading", drivetrain.getHeadingDegrees());
    SmartDashboard.putNumber("Left Wheel Speed", drivetrain.getLeftMotorSpeed());
    SmartDashboard.putNumber("Right Wheel Speed", drivetrain.getRightMotorSpeed()); */

    SmartDashboard.putNumber("Pigeon Yaw", drivetrain.pigeon.getYaw());
    SmartDashboard.putNumber("Pigeon Pitch", drivetrain.pigeon.getPitch());
    SmartDashboard.putNumber("Pigeon Roll", drivetrain.pigeon.getRoll());

    SmartDashboard.putBoolean("Wrist Switch", wrist.getWristSwitchState());

    SmartDashboard.putNumber("Arm Position", arm.getArmPosition());
    SmartDashboard.putNumber("Arm Position 2", arm.getArmPosition());
    SmartDashboard.putNumber("Slide Position", arm.getSlidePosition());
    SmartDashboard.putNumber("Slide Position2", arm.getSlidePosition());
    SmartDashboard.putNumber("Wrist Position", wrist.getWristPosition());
    SmartDashboard.putNumber("Wrist Rotate Position", wrist.getWristRotatePosition());
    SmartDashboard.putNumber("Left Drive Position", drivetrain.getLeftDrivePosition());
    SmartDashboard.putNumber("Right Drive Position", drivetrain.getRightDrivePosition());

    SmartDashboard.putNumber("Wrist Velocity", wrist.getWristRotateVelocity());
    SmartDashboard.putNumber("Wrist Angle Velocity", wrist.getWristAngleVelocity());
    SmartDashboard.putNumber("Arm Velocity", arm.getArmVelocity());
    SmartDashboard.putNumber("Slide Velocity", arm.getArmSlideVelocity());
    SmartDashboard.putNumber("Slide Velocity2", arm.getArmSlideVelocity());

    SmartDashboard.putNumber("Arm Error", arm.armAngleLead.getClosedLoopError(0));
    SmartDashboard.putNumber("Slide Error", arm.armSlide.getClosedLoopError(0));

   // SmartDashboard.putNumber("Slide Switch", arm.armSlide.getSensorCollection().isFwdLimitSwitchClosed());

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
  //  m_autonomousCommand = new TestPath(autoTrajectory);
    //drivetrain.zeroOdometry(); //remove
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    m_teleopCommand.schedule();  

    wrist.setDefaultCommand(new RotateWristWithJoystick());
    arm.setDefaultCommand(new MoveArmWithJoystick());

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
