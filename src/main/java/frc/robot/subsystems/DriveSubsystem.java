/* */
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

import java.net.FileNameMap;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.PWM1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.fasterxml.jackson.databind.type.MapType;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.jni.AHRSJNI;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import frc.robot.subsystems.Vision.Cameras;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Encoder;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class DriveSubsystem extends SubsystemBase {
  // private final WPI_TalonSRX leftLeader;
  // private final WPI_TalonSRX leftFollower;
  // private final WPI_TalonSRX rightLeader;
  // private final WPI_TalonSRX rightFollower;

  private Pose2d oldRobotPose = new Pose2d(); 
  private long oldTimeSample = 0;
  private int count = 0;
  /** the robots speed if meters per sec */
  private Pose2d robotSpeed;

  
  //private final Solenoid Shifter;
  
  private final TalonFX leftLeader;
  private final TalonFX leftFollower;
  private final TalonFX rightLeader;
  private final TalonFX rightFollower;

  private final AHRS gyroThePirate;
  private final Vision vision;
  private Field2d field;

  private final PIDController pidController;
  private final DifferentialDrive drive;
  private final DifferentialDrivePoseEstimator poseEstimator;
  //private final Encoder leftEncoder;
  Encoder rightEncoder;
  
  private static final double cpr = 360; //if am-3132
  

  private static final double whd = 6; // for 6 inch wheel

  /**
   * 
   */
  public DriveSubsystem() {
     
    //Shifter = new Solenoid(PneumaticsModuleType.REVPH, 0);

    leftLeader = new TalonFX(1);
    leftFollower = new TalonFX(2);

    rightLeader = new TalonFX(5);
    rightFollower = new TalonFX(4);

    TalonFXConfiguration leftConfiguration = new TalonFXConfiguration();
    TalonFXConfiguration rightConfiguration = new TalonFXConfiguration();

    // need to know if need to invert same side motors
    leftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var leftLeaderConfigurator = leftLeader.getConfigurator();
    var leftFollowerConfigurator = leftFollower.getConfigurator();
    var rightLeaderConfigurator = rightLeader.getConfigurator();
    var rightFollowerConfigurator = rightFollower.getConfigurator();

    var limitConfigs = new CurrentLimitsConfigs();
    // enable stator current limit
    limitConfigs.StatorCurrentLimit = 80;
    limitConfigs.SupplyCurrentLimit = 40;
    limitConfigs.StatorCurrentLimitEnable = true;
    

    leftLeaderConfigurator.apply(limitConfigs);
    leftFollowerConfigurator.apply(limitConfigs);
    rightLeaderConfigurator.apply(limitConfigs);
    rightFollowerConfigurator.apply(limitConfigs);
    
    leftLeaderConfigurator.apply(leftConfiguration);
    leftFollowerConfigurator.apply(leftConfiguration);
    rightLeaderConfigurator.apply(rightConfiguration);
    rightFollowerConfigurator.apply(rightConfiguration);

    /* Set up followers to follow leaders */
    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    rightFollower.setControl(new Follower(rightLeader.getDeviceID(), MotorAlignmentValue.Aligned));
  
    leftLeader.setSafetyEnabled(true);
    rightLeader.setSafetyEnabled(true);

    // // //create brushed motors for drive
    // leftLeader = new WPI_TalonSRX(1);
    // leftFollower = new WPI_TalonSRX(3);

    // rightLeader = new WPI_TalonSRX(4);
    // rightFollower = new WPI_TalonSRX(2);

    // leftFollower.follow(leftLeader);
    // rightFollower.follow(rightLeader);

 
    // leftFollower.setInverted(true);
    // leftLeader.setInverted(true);

    //leftEncoder = new Encoder(2,1);
    //leftEncoder.setDistancePerPulse(Math.PI*whd/cpr); //distance per pulse is pi* (wheel diameter / counts per revolution)
    rightEncoder = new Encoder(3,4);
    rightEncoder.setDistancePerPulse(Math.PI*whd/cpr); //distance per pulse is pi* (wheel diameter / counts per revolution)

    gyroThePirate = new AHRS(NavXComType.kMXP_SPI);
    
    pidController = new PIDController(0.75, 0, 0);

    field = new Field2d();
        
    // set up differential drive class
    drive = new DifferentialDrive(leftLeader::set, rightLeader::set);

    poseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(29),
                                                  new Rotation2d(Math.toRadians(gyroThePirate.getAngle())),
                                                  /*leftEncoder.getDistance()*/0, 
                                                  rightEncoder.getDistance(), 
                                                  new Pose2d(4, 0, new Rotation2d(0)));
    Supplier<Pose2d> poseSupplier = () -> poseEstimator.getEstimatedPosition();
    vision = new Vision(poseSupplier, field);
    // // Set can timeout. Because this project only sets parameters once on
    // // construction, the timeout can be long without blocking robot operation. Code
    // // which sets or gets parameters during operation may need a shorter timeout.
    // leftLeader.setCANTimeout(250);
    // rightLeader.setCANTimeout(250);
    // leftFollower.setCANTimeout(250);
    // rightFollower.setCANTimeout(250);

    // // Create the configuration to apply to motors. Voltage compensation
    // // helps the robot perform more similarly on different
    // // battery voltages (at the cost of a little bit of top speed on a fully charged
    // // battery). The current limit helps prevent tripping
    // // breakers.
    // SparkMaxConfig config = new SparkMaxConfig();
    // config.voltageCompensation(12);
    // config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    // // Set configuration to follow each leader and then apply it to corresponding
    // // follower. Resetting in case a new controller is swapped
    // // in and persisting in case of a controller reset due to breaker trip
    // config.follow(leftLeader);
    // leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // config.follow(rightLeader);
    // rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // // Remove following, then apply config to right leader
    // config.disableFollowerMode();
    // rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // // Set config to inverted and then apply to left leader. Set Left side inverted
    // // so that postive values drive both sides forward
    // config.inverted(true);
    // leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    poseEstimator.update(new Rotation2d(gyroThePirate.getAngle()),
                          /*leftEncoder.getDistance()*/0, 
                          rightEncoder.getDistance());
                        
    vision.updatePoseEstimation(poseEstimator);
    count++;
    if(count == 6){
      robotSpeed = getRobotFastness();
      SmartDashboard.putNumber("Y speed", robotSpeed.getY());
      SmartDashboard.putNumber("X speed", robotSpeed.getX());
      count = 0;
    }
    
    SmartDashboard.putNumber("od Y", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("od X", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("angle of gyro", gyroThePirate.getAngle());
    
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  
  }

  /**
   * to be called at the begining of auto
   * 
   * @param poseMeters
   */
  public void setRobotPose(Pose2d poseMeters){
    poseEstimator.resetPose(poseMeters);
  }
  /**
   * field centric drive 
   * 
   * @param xSpeed foward
   * @param ySpeed side
   * @param zRotation
   */
  public void fieldCentricDrive(double xSpeed, double ySpeed, double xRotation, double yRotation){
    double rotation = 0;
    double currentAngle = Math.toRadians(gyroThePirate.getAngle());
    double error = angleSubtractRadians(Math.atan2(ySpeed, xSpeed), currentAngle);
    double robotSpeed = Math.sqrt((xSpeed*xSpeed) +(ySpeed*ySpeed)) * Math.cos(error);
    if(Math.abs(error) > Math.PI/2){
        error = angleSubtractRadians(Math.atan2(ySpeed, xSpeed) + Math.PI, currentAngle);
        
    }
    //error = angleSubtractRadians(Math.atan2(yRotation, xRotation), currentAngle);
    rotation = pidController.calculate(currentAngle, error + currentAngle);

    drive.arcadeDrive(robotSpeed, rotation);
  }

  /**
   * 
   * @param x
   * @param y
   * @return
   */
  public static double floorMod(double x, double y) {
    double result = (x - Math.floor(x / y) * y);
    return result == y ? 0 : result;
  }
  /**
   * Returns the difference of two angles considering wrap.
   * Subracts angle2 from angle1
   * @param angle1 in degrees
   * @param angle2 in degrees
   * @return degrees value between -180 and +180
   */
  public static double angleSubtractDegrees(double angle1, double angle2){
    return floorMod((angle1-angle2)+180, 360)-180;
  }

    /**
   * Returns the difference of two angles considering wrap.
   * Subracts angle2 from angle1
   * @param angle1 in rotations
   * @param angle2 in rotations
   * @return rotations value between -0.5 and +0.5
   */
  public static double angleSubtractRotations(double angle1, double angle2){
    return floorMod((angle1-angle2)+0.5, 1)-0.5;
  }

  /**
   * Returns the difference of two angles considering wrap.
   * Subracts angle2 from angle1
   * @param angle1 in radians
   * @param angle2 in radians
   * @return radian value between -pi and + pi
   */
  public static double angleSubtractRadians(double angle1, double angle2){
    return floorMod((angle1-angle2)+Math.PI, 2*Math.PI)-Math.PI;
  }
  
  public Pose2d getRobotPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public double getRobotAngleGyro(){
    return gyroThePirate.getAngle();
  }

  public void autoShiftGears(){
    //double speed = Math.abs(Math.sqrt((robotSpeed.getX()*robotSpeed.getX()) + (robotSpeed.getY()*robotSpeed.getY())));
    // if(speed > 3.0){
    //   Shifter.set(true);
    //   
    // }
    // if(speed < 2.0){
    //   Shifter.set(false);
    //   
    // }
  }

  public void shiftToHigh(){
    //Shifter.set(true);
  }

  public void shiftToLow(){
    //Shifter.set(false);
  }


  /**
   * returns the speed if meters per sec
   * 
   * @return returns field centric pose2d for speed 
   */
  public Pose2d getRobotFastness(){
    double mult = 1000/(System.currentTimeMillis() - oldTimeSample);
    Pose2d britishYardsASecond = new Pose2d(
                                            (poseEstimator.getEstimatedPosition().getX() - oldRobotPose.getX())*mult,
                                            (poseEstimator.getEstimatedPosition().getY() - oldRobotPose.getY())*mult,
                                            new Rotation2d(0));
    
    oldTimeSample = System.currentTimeMillis();
    oldRobotPose = poseEstimator.getEstimatedPosition();
    return britishYardsASecond;
    
  }

  public Pose2d minPose2ds(Pose2d pose1, Pose2d pose2){
    return new Pose2d(pose1.getX() - pose2.getX(), pose1.getY() - pose2.getY(), 
    new Rotation2d(pose1.getRotation().getRadians() - pose2.getRotation().getRadians()));
  }
 
  public Command turnToAngle(double angle){
    return run(
        () -> {
      double dalekSpinnnnSpeeeed = -0.005;
      double neededExterminateCalculator = (((angleSubtractDegrees(gyroThePirate.getAngle(), angle)) + 180) % 360) - 180;
      double re = neededExterminateCalculator * dalekSpinnnnSpeeeed;
      drive.arcadeDrive(0, constrain(1, -1, (Math.abs(re)/re)*(0.23+Math.abs(re))));
    });
  }

    /**
   * 
   * @param hi
   * @param low
   * @param input
   * @return
   */
  public double constrain(double hi, double low, double input){
      if(input > hi){
          input = hi;
      } else if(input < low){
          input = low;
      }
      return input;
  }

}