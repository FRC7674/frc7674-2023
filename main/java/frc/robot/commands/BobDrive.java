package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.utils.DriveMode;
import frc.robot.utils.DriveSignal;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.BobDriveHelper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class BobDrive extends CommandBase {

	BobDriveHelper helper;
	private double quickTurnThreshold = 0.95; // changed
	private double deadband = 0.007; // changed

	private RobotContainer robotContainer = new RobotContainer();
	//private PIDController limelightRotatePID = new PIDController(0.25, 0.01, 0.0);

	public BobDrive() {
		addRequirements(Robot.drivetrain);
		helper = new BobDriveHelper();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	public void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {

		double rotateValue = robotContainer.getRightStick().getFirst() * 0.11;
		double moveValue = robotContainer.getLeftStick().getSecond() * 1.0;
		moveValue = Math.abs(moveValue) > deadband ? moveValue : 0.0;
		rotateValue = Math.abs(rotateValue) > deadband ? rotateValue : 0.0;

		//precision mode: if left bumper is pressed then turn is divided by 2
		if (robotContainer.m_driverController.leftBumper().getAsBoolean() == true) {
			rotateValue = rotateValue / 2;
			moveValue = moveValue / 5;
		}

		boolean quickTurn = (moveValue < quickTurnThreshold && moveValue > -quickTurnThreshold);	
		DriveSignal driveSignal = helper.cheesyDrive(moveValue, rotateValue, quickTurn, false);
		Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
	}

	public boolean isFinished() {
		return false;
	}

	public void end() {
	}

}
