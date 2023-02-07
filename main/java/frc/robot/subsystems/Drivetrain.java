// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.commands.BobDrive;
import frc.robot.utils.DriveSignal;


public class Drivetrain extends SubsystemBase {

  // Motors
 // public TalonFX leftLead = new TalonFX(2);
 // public TalonFX leftFollow1 = new TalonFX(1);

  public CanSpark leftLead = new Spark(0);
  public Spark leftfollow = new Spark(0);
  

 // public TalonFX rightLead = new TalonFX(5);
 // public TalonFX rightFollow1 = new TalonFX(4);

  public Spark rightLead = new Spark(0);
  public Spark rightFollow = new Spark(0);

  public WPI_Pigeon2 pigeon = new WPI_Pigeon2(6);
  
  
  private DifferentialDriveOdometry odometry;
  Rotation2d heading; //= new Rotation2d(Units.degreesToRadians(pigeon.getFusedHeading()));

  private static final double rampRate = 0.25;

  //DifferentialDrive differentialDrive = new DifferentialDrive(leftLead, rightLead);

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    setMotorConfigsToDefault();
    setMotorInversions();
    setMotorNeutralModes();
    setMotorRampRates();
    

    //TODO reset encoders
    odometry = new DifferentialDriveOdometry(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters());

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters());
  }

  public void drive(ControlMode controlMode, double left, double right) {
    //Left control mode is set to left
    this.leftLead.set(controlMode, left);

    //Right control mode is set to right
    this.rightLead.set(controlMode, right);
  }
  
  public void drive(ControlMode controlMode, DriveSignal driveSignal) {
    this.drive(controlMode, driveSignal.getLeft(), driveSignal.getRight());
  }

  public void setFollowers() {
    leftFollow.follow(leftLead);


    rightFollow.follow(rightLead);

  }

  private void setMotorConfigsToDefault() {
    
    leftLead.configFactoryDefault();
    leftFollow.configFactoryDefault();

    
    rightLead.configFactoryDefault();
    rightFollow.configFactoryDefault();
 
  }

  private void setMotorInversions() {
    
    leftLead.setInverted(false);
    leftFollow.setInverted(false);
 

    rightLead.setInverted(true);
    rightFollow.setInverted(true);
  
  }

  private void setMotorNeutralModes() {
    
    leftLead.setNeutralMode(NeutralMode.Coast);
    leftFollow.setNeutralMode(NeutralMode.Coast);
 

    rightLead.setNeutralMode(NeutralMode.Coast);
    rightFollow.setNeutralMode(NeutralMode.Coast);

  }

  private void setMotorRampRates() {

    leftLead.configOpenloopRamp(rampRate);
    leftFollow.configOpenloopRamp(rampRate);


    rightLead.configOpenloopRamp(rampRate);
    rightFollow.configOpenloopRamp(rampRate);

  }

  public double getLeftLeadDriveDistanceMeters() {
    return this.leftLead.getSelectedSensorPosition() * Constants.DriveConstants.metersPerEncoderTick;
  }

  public double getRightLeadDriveDistanceMeters() {
    return this.rightLead.getSelectedSensorPosition() * Constants.DriveConstants.metersPerEncoderTick;
  }

  public double getLeftLeadDriveDistanceTicks() {
    return this.leftLead.getSelectedSensorPosition();
  }

  public double getRightLeadDriveDistanceTicks() {
    return this.rightLead.getSelectedSensorPosition();
  }

  public Rotation2d getHeading() {
    heading = Rotation2d.fromDegrees(0);
    // heading = Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.getRotation2d().getDegrees(), -180, 180));
    return heading;
  }

  public double getHeadingDegrees() {
    return 0; //MathUtil.inputModulus(pigeon.getRotation2d().getDegrees(), -180, 180);
  }

  public void resetHeading() {
    //pigeon.setFusedHeading(0);
    this.getHeading();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getLeftMotorSpeed() {
    double leftmotorspeed = leftLead.getSelectedSensorVelocity() * Constants.DriveConstants.metersPerEncoderTick;
    return leftmotorspeed;
  }

  public double getRightMotorSpeed() {
    double rightmotorspeed = rightLead.getSelectedSensorVelocity() * Constants.DriveConstants.metersPerEncoderTick;
    return rightmotorspeed;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() { // TODO : These should be scaled with the distance travelled per 'pulse'
    	return new DifferentialDriveWheelSpeeds(getLeftMotorSpeed(), getRightMotorSpeed());
  }

  public void zeroOdometry() {
    resetEncoders();
    resetHeading();

    Pose2d origin = new Pose2d(0,0,new Rotation2d(0));
    odometry.resetPosition(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters(), origin);
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters(), pose);
  }

  public void resetEncoders() {
    
    leftLead.setSelectedSensorPosition(0);
    leftFollow.setSelectedSensorPosition(0);
  

    rightLead.setSelectedSensorPosition(0);
    rightFollow.setSelectedSensorPosition(0);
  
  }

  public void tankDriveVolts(double leftVoltage, double rightVoltage) {
    leftLead.set(TalonFXControlMode.PercentOutput, leftVoltage);
    rightLead.set(TalonFXControlMode.PercentOutput, rightVoltage);
  }

}

