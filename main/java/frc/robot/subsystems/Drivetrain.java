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
import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
  // conflicts with the rev robotics motor type \\
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType; 
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.commands.BobDrive;
import frc.robot.utils.DriveSignal;
 
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;


 
public class Drivetrain extends SubsystemBase {

  // Motors
      // Left lead and follow motors
  public CANSparkMax LeftDriveLead = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax LeftDriveFollow1 = new CANSparkMax(2, MotorType.kBrushless);
  public CANSparkMax LeftDriveFollow2 = new CANSparkMax(3, MotorType.kBrushless);

  // Right lead and follow motors
  public CANSparkMax RightDriveLead = new CANSparkMax(4, MotorType.kBrushless);
  public CANSparkMax RightDriveFollow1 = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax RightDriveFollow2 = new CANSparkMax(6, MotorType.kBrushless);

  public WPI_Pigeon2 pigeon = new WPI_Pigeon2(13);

// Drive Distance for encoders
  public RelativeEncoder leftDriveEncoder = LeftDriveLead.getEncoder();
  public SparkMaxPIDController leftDrivePidController = LeftDriveLead.getPIDController();
  // Drive Distance for enconders
  public RelativeEncoder rightDriveEncoder = RightDriveLead.getEncoder();
  public SparkMaxPIDController rightDrivePidController = RightDriveLead.getPIDController();
  
  private DifferentialDriveOdometry odometry;
  Rotation2d heading; //= new Rotation2d(Units.degreesToRadians(pigeon.getFusedHeading()));

  public boolean IsSlowMode = false;

  private static final double rampRate = 0.25;

  DifferentialDrive differentialDrive = new DifferentialDrive(LeftDriveLead, RightDriveLead);
  

  /** Creates a new Drivetrain. */
   public Drivetrain() {

    //setMotorConfigsToDefault();
    setMotorInversions();
    setMotorNeutralModes(IdleMode.kBrake);
    setMotorRampRates();
    setFollowers();

    //TODO reset encoders
    odometry = new DifferentialDriveOdometry(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters());
    
    /* 
    // trying to fix the error we get in driverstation
    differentialDrive.setExpiration(.30);
    differentialDrive.feed();
    */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters());
  }

  public void drive(ControlMode controlMode, double left, double right) {
    //Left control mode is set to left
    this.LeftDriveLead.set(left);
    //this.LeftDriveFollow1.set(left);
    //this.LeftDriveFollow2.set(left);

    //Right control mode is set to right
    this.RightDriveLead.set(right);
    //this.RightDriveFollow1.set(right);
    //this.RightDriveFollow2.set(right);
  }
  
  public void drive(ControlMode controlMode, DriveSignal driveSignal) {
    this.drive(controlMode, driveSignal.getLeft(), driveSignal.getRight());
  }

//SET MOTORS TO DEFAULT SETTINGS, RUNS ONCE, FIRST STEP
  private void setMotorConfigsToDefault() {
    LeftDriveLead.restoreFactoryDefaults();
    LeftDriveFollow1.restoreFactoryDefaults();
    LeftDriveFollow2.restoreFactoryDefaults();
    
    RightDriveLead.restoreFactoryDefaults();
    RightDriveFollow1.restoreFactoryDefaults();
    RightDriveFollow2.restoreFactoryDefaults();
  }

    // Sets followers to lead
    public void setFollowers() {
      LeftDriveFollow1.follow(LeftDriveLead); 
      LeftDriveFollow2.follow(LeftDriveLead);
  
      RightDriveFollow1.follow(RightDriveLead);
      RightDriveFollow2.follow(RightDriveLead);
    }


//SETS INVERSIONS SO MOTORS SPIN THE RIGHT WAY, RUNS ONCE, SECOND STEP
  private void setMotorInversions() {
    LeftDriveLead.setInverted(false);
    LeftDriveFollow1.setInverted(false);
    LeftDriveFollow2.setInverted(false);
 

    RightDriveLead.setInverted(true);
    RightDriveFollow1.setInverted(true);
    RightDriveFollow2.setInverted(true);
  }

//SETS TO BRAKE MODE, RUNS ONCE, THIRD STEP
  public void setMotorNeutralModes(IdleMode Mode) {
    
    LeftDriveLead.setIdleMode(Mode);
    LeftDriveFollow1.setIdleMode(Mode);
    LeftDriveFollow2.setIdleMode(Mode);

    RightDriveLead.setIdleMode(Mode);
    RightDriveFollow1.setIdleMode(Mode);
    RightDriveFollow2.setIdleMode(Mode);
  }

//SETS SPEED? RUNS ONCE, FOURTH STEP
  private void setMotorRampRates() {

    LeftDriveLead.setOpenLoopRampRate(rampRate);
    LeftDriveFollow1.setOpenLoopRampRate(rampRate);
    LeftDriveFollow2.setOpenLoopRampRate(rampRate);


    RightDriveLead.setOpenLoopRampRate(rampRate);
    RightDriveFollow1.setOpenLoopRampRate(rampRate);
    RightDriveFollow2.setOpenLoopRampRate(rampRate);
  }


  public double getLeftLeadDriveDistanceMeters() {
    return this.LeftDriveLead.getEncoder().getPosition() * Constants.DriveConstants.metersPerEncoderTick;
  }

  public double getRightLeadDriveDistanceMeters() {
    return this.RightDriveLead.getEncoder().getPosition() * Constants.DriveConstants.metersPerEncoderTick;
  }

  public double getLeftLeadDriveDistanceTicks() {
    return this.LeftDriveLead.getEncoder().getPosition();
  }

  public double getRightLeadDriveDistanceTicks() {
    return this.RightDriveLead.getEncoder().getPosition();
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
    double leftmotorspeed = LeftDriveLead.getEncoder().getVelocity() * Constants.DriveConstants.metersPerEncoderTick;
    return leftmotorspeed;
  }

  public double getRightMotorSpeed() {
    double rightmotorspeed = RightDriveLead.getEncoder().getVelocity() * Constants.DriveConstants.metersPerEncoderTick;
    return rightmotorspeed;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() { // TODO : These should be scaled with the distance travelled per 'pulse'
    	return new DifferentialDriveWheelSpeeds(getLeftMotorSpeed(), getRightMotorSpeed());
  }

     public void zeroOdometry() {
    resetEncoders();
    resetHeading(); 
    
  }
    Pose2d origin = new Pose2d(0,0,new Rotation2d(0));
//   odometry.resetPosition(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters(), origin);
 // }

   public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters(), pose);
  } 

   public void resetEncoders() {
    
    LeftDriveLead.getEncoder().getPosition();
    LeftDriveFollow1.getEncoder().getPosition();
    LeftDriveFollow2.getEncoder().getPosition();
  

    RightDriveLead.getEncoder().getPosition();
    RightDriveFollow1.getEncoder().getPosition();
    RightDriveFollow2.getEncoder().getPosition();
  
  }

  public void tankDriveVolts(double leftVoltage, double rightVoltage) {
    LeftDriveLead.set(leftVoltage);
    RightDriveLead.set(rightVoltage);
  } 

  public double getLeftDrivePosition(){
    return LeftDriveLead.getEncoder().getPosition();
  }

  public double getRightDrivePosition(){
    return RightDriveLead.getEncoder().getPosition();
  }

// Drive Distance
  public void setLeftDrivePosition(double Position) {
    leftDrivePidController.setReference(Position, CANSparkMax.ControlType.kPosition);
  }

// Drive Distance
  public void setRightDrivePosition(double Position) {
    rightDrivePidController.setReference(Position, CANSparkMax.ControlType.kPosition);
  }

// Drive Distance
  public void setDrivetrainPositionToZero() {
    setLeftDrivePosition(0);
    setRightDrivePosition(0);
  }

  public void SetIsSlowMode(boolean IsSlow){
    IsSlowMode = IsSlow;
  }

}
 

