// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.Constants.FuelConstants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;

import javax.lang.model.util.ElementScanner14;


public class Turret extends SubsystemBase {
  private final SparkMax angleOfTurretSparkMax;
  private final SparkMax leftFlyWheelSparkMax;
  private final SparkMax rightFlyWheelSparkMax;
  private final DutyCycleEncoder turretEncoder;
  private Pose2d hub;
  private PIDController angleOfTurretPIDController;
  private PIDController flyWheelPidController;
  private ShooterHood hood;
  private char allianceColor;
  private char activeHub;
  private boolean isHubActive;
  private boolean shouldItTrack;
  private boolean ifInManual;
  /** in rotations of the motor */
  private double desiredAngleOfTurret;
  private boolean ifInZone;
  private double positionOfT;

  private Timer timer = new Timer();

  /** the robots current Pose2d */
  private Pose2d robotPose;
  /** the robot Pose Corrected For Robot Speed */
  private Pose2d correctedRobotSpeed;

  /**What the robot should shoot at. like the hub and passing spots*/
  private Pose2d target;

  /** Creates a new CANBallSubsystem. */
  public Turret() {
    ifInManual = false;
    //System.out.println("sdfgTurres");
    robotPose = new Pose2d();
    positionOfT = 0;
    // create brushed motors for each of the motors on the launcher mechanism
    leftFlyWheelSparkMax = new SparkMax(Left_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    rightFlyWheelSparkMax = new SparkMax(Right_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    angleOfTurretSparkMax = new SparkMax(Angle_MOTOR_ID, MotorType.kBrushless);

    angleOfTurretPIDController = new PIDController(0.1, 0, 0);
    flyWheelPidController = new PIDController(0.1, 0, 0);

    turretEncoder = new DutyCycleEncoder(0);

    hood = new ShooterHood();
    
    //hub = BLUE_HUB;
    // create the configuration for the angle motor, set a current limit and apply
    // the config to the controller
    SparkMaxConfig angleConfig = new SparkMaxConfig();
    angleConfig.smartCurrentLimit(Angle_MOTOR_CURRENT_LIMIT);
    angleOfTurretSparkMax.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.smartCurrentLimit(Left_MOTOR_CURRENT_LIMIT);
    leftConfig.follow(Right_LAUNCHER_MOTOR_ID, true);
    leftFlyWheelSparkMax.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.smartCurrentLimit(Right_MOTOR_CURRENT_LIMIT);
    rightFlyWheelSparkMax.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    
    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    // SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    // SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    // SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    // SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    // SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  @Override
  public void periodic() {
    setAngle(desiredAngleOfTurret);
    SmartDashboard.putNumber("turret encoder rotations", turretEncoder.get());
    SmartDashboard.putBoolean("if in manual mode", ifInManual);
    SmartDashboard.putNumber("angle of turret", angleOfTurretSparkMax.getEncoder().getPosition());
    // This method will be called once per scheduler run
    

    if (allianceColor == 'B'){
      //if in alliance zone
      if(robotPose.getX() < 4.5){
        target = hub;
        ifInZone = true;
        if(isHubActive){
          shouldItTrack = true;
        } else{
          shouldItTrack = false;
        }
      } else{
        ifInZone = false;
        shouldItTrack = true;
        if(robotPose.getY() < 4){
          target = new Pose2d(3, 2, new Rotation2d());
        } else{
          target = new Pose2d(3, 6, new Rotation2d());
        }
      }
    }
    else if (allianceColor == 'R'){
      //if in alliance zone
      if(robotPose.getX() > 12){
        target = hub;
        ifInZone = true;
        if(isHubActive){
          shouldItTrack = true;
        } else{
          shouldItTrack = false;
        }
      } else{
        ifInZone = false;
        shouldItTrack = true;
        if(robotPose.getY() < 4){
          target = new Pose2d(13.5, 2, new Rotation2d());
        } else{
          target = new Pose2d(13.5, 6, new Rotation2d());
        }
      }
    }
    
    correctedRobotSpeed = new Pose2d(
                                    robotPose.getX() + SmartDashboard.getNumber("X speed", LAUNCHING_LAUNCHER_VOLTAGE),
                                    robotPose.getX() + SmartDashboard.getNumber("X speed", LAUNCHING_LAUNCHER_VOLTAGE),
                                    robotPose.getRotation());

    // code for finding if hub is active 
    String gameData;
    gameData = DriverStation.getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
          //Blue case code
          if (timer.hasElapsed(10)){
            activeHub = 'R';
          }else if(timer.hasElapsed(35)){
            activeHub = 'B';
          }else if(timer.hasElapsed(60)){
            activeHub = 'R';
          }else if(timer.hasElapsed(85)){
            activeHub = 'B';
          }else if(timer.hasElapsed(110)){
            activeHub = 'R';
          }else if(timer.hasElapsed(140)){
            activeHub = 'B';
          }else{
            activeHub = 'B';
          }
          break;
        case 'R' :
          //Red case code
          if (timer.hasElapsed(10)){
            activeHub = 'B';
          }else if(timer.hasElapsed(35)){
            activeHub = 'R';
          }else if(timer.hasElapsed(60)){
            activeHub = 'B';
          }else if(timer.hasElapsed(85)){
            activeHub = 'R';
          }else if(timer.hasElapsed(110)){
            activeHub = 'B';
          }else if(timer.hasElapsed(140)){
            activeHub = 'R';
          }else{
            activeHub = 'R';
          }
          break;
        default :
          //This is corrupt data
          break;
      }
    } else {
      //Code for no data received yet
      isHubActive = true;
      shouldItTrack = true;
    }
    if(activeHub == allianceColor){
      isHubActive = true;
    }else{
      isHubActive = false;
    }
  }

  /**
  * to be called in auto Mouse init
  */
  public void findAllianceColor(){
    // code to find what target to aim at.  and if we should track it
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            //<RED ACTION>
            hub = RED_HUB;
            allianceColor = 'R';

            
        }
        if (ally.get() == Alliance.Blue) {
            //<BLUE ACTION>
            hub = BLUE_HUB;
            allianceColor = 'B';
        }
    }
    else {
        //<NO COLOR YET ACTION>
        isHubActive = true;
        shouldItTrack = true;

    }
  }
  
  public char getAllianceColor(){
    return allianceColor;
  }
  /**
   * a method to set the rpm of shooter
   * 
   * @param speed desired shooter speed
   * @return returns the shooter real speed rpms
   */
  public double setFlyWheelSpeed(double speed) {
    rightFlyWheelSparkMax.set(flyWheelPidController.calculate(rightFlyWheelSparkMax.getEncoder().getVelocity(), speed));
    return rightFlyWheelSparkMax.getEncoder().getVelocity();
  }


  // A method to set the voltage of the intake roller
  /**
   * should be in rotations for turret
   * 
   * @param angle
   */
  public void setAngle(double rotations) {

    angleOfTurretSparkMax.set(angleOfTurretPIDController.calculate(angleOfTurretSparkMax.getEncoder().getPosition(), constrain(193.75, -193.75, rotations)));
    //angleOfTurretSparkMax.set(angleOfTurretPIDController.calculate(angleOfTurretSparkMax.getEncoder().getPosition(), constrain(-11.78, -12.17, rotations)));
  }




  /**
   * ramps up to speed and then feeds and agitates the balls with the intake
   * 
   * @param feeder feeder
   * @param intake intake
   */
  public Command shoot(Feeder feeder, Intake intake){
    return run(
        () -> {
      // if(shouldItTrack){
      //   double desiredSpeed = getFlywheelSpeed(getDistanceBetween(correctedRobotSpeed, target));
      //   double realSpeed = setFlyWheelSpeed(desiredSpeed);
      //   if (Math.abs(desiredSpeed - realSpeed) < (0.54864/getDistanceBetween(correctedRobotSpeed, target)) & !ifInManual){
      //     feeder.setFeedSpeed(0.2);
      //     intake.setHungrySpeed(0.1);
      //   }
      // }
      feeder.setFeedSpeed(-1);
      intake.setHungrySpeed(0.3);
    });

  }
  /**
   * for auto
   * 
   * @param feeder
   * @param intake
   */
  public void rampUpAndAlignAndShoot(Feeder feeder, Intake intake){
    double desiredSpeed = getFlywheelSpeed(getDistanceBetween(robotPose, target));
    double realSpeed = setFlyWheelSpeed(desiredSpeed);
    if (Math.abs(desiredSpeed - realSpeed) < (0.54864/getDistanceBetween(robotPose, target)) & !ifInManual){
      feeder.setFeedSpeed(0.2);
      intake.setHungrySpeed(0.1);
    }
    trackPose2d(robotPose, hub);

    if(Math.abs(getHoodAngle(getDistanceBetween(robotPose, hub))-hood.getAngle()) > 5){
          hood.setAngle(getHoodAngle(getDistanceBetween(robotPose, hub)));
        }

  }


  /**
   * Tracks the target
   * 
   * @param currentPose current robot pose
   */
  public void trackTarget(){
    if(shouldItTrack & !ifInManual){
      trackPose2d(correctedRobotSpeed, target);
    }
    if (ifInZone & isHubActive  & !ifInManual){
        if(Math.abs(getHoodAngle(getDistanceBetween(correctedRobotSpeed, target))-hood.getAngle()) > 5){
          hood.setAngle(getHoodAngle(getDistanceBetween(correctedRobotSpeed, target)));
        }
    } else{
      if(!ifInManual){
        hood.setAngle(67);
      }
    }

  }

  /**
   * 
   * @param currentPose
   * @param target
   */
  public void trackPose2d(Pose2d currentPose, Pose2d target){
    double x = (currentPose.getX()+Math.cos(currentPose.getRotation().getRadians()+angleToTurret))*centerOfRobotToTurret;
    double y = (currentPose.getY()+Math.cos(currentPose.getRotation().getRadians()+angleToTurret))*centerOfRobotToTurret;
    double angle = currentPose.getRotation().getRadians() + (angleOfTurretSparkMax.getEncoder().getPosition()*(1/10)*(1/25));
    Pose2d turretFieldCentricPose = new Pose2d(x, y, new Rotation2d(angle));
    desiredAngleOfTurret = ((Math.atan2(x-target.getX(), y-target.getY()) *(1/10)*(1/25))*Math.PI*2);

  }

  /**
   * gets desired fly wheel speed
   * 
   * @param dis distance in meters from the target
   * @return return rpm
   */
  public double getFlywheelSpeed(double dis){
    return (6.56+0.0725*dis+0.0606*Math.pow(dis, 2))*30/Math.PI/0.0254;
  }
  /**
   * return the desired hood angle
   * 
   * @param dis distance in meters from the target
   * @return return in degrees
   */
  public double getHoodAngle(double dis){
    return 89.9*Math.log(-0.111*dis);
  }

  // A method to stop the rollers
  public void stop() {
    //feederRoller.set(0);
    //leftFlyWheelSparkMax.set(0);
  }

  /**
   * find the distance between two pose2ds
   * 
   * @param firstPosition
   * @param secondPosition
   * @return  returns the hypotenuse
   */
  public double getDistanceBetween(Pose2d firstPosition, Pose2d secondPosition){
    double xDifference = firstPosition.getX()-secondPosition.getX();
    double yDiffference = firstPosition.getY()-secondPosition.getY();
    return Math.abs(Math.sqrt((xDifference*xDifference)+(yDiffference*yDiffference)));
  }
  
  public void joyStickServo(double dis){
    if(ifInManual){  
      hood.setAngle((dis*0.5)+hood.getAngle());
    }
  } 

  /**
   * MUST BE CALLED Periodically
   * 
   * @param currentPose
   */
  public void updateRobotPose(Pose2d currentPose){
    robotPose = currentPose;
  }
  /**
   * starts the timer
   */
  public void setUpTimer(){
    timer.start();
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

  public void resetAngleOfTurret(){
    double offSet = 0;
    double resetAngle = 25.0*angleSubtractRotations(turretEncoder.get(), offSet);
    //SmartDashboard.putNumber("angle of turret is being set to", 25.0*angleSubtractRotations(turretEncoder.get(), offSet));
    //angleOfTurretSparkMax.getEncoder().setPosition(25.0*angleSubtractRotations(turretEncoder.get(), offSet));
    //System.out.println("Turret angle reset to: " + resetAngle);
    SmartDashboard.putNumber("resetAngle", resetAngle);
    angleOfTurretSparkMax.getEncoder().setPosition(resetAngle);
  }



  public Command rightPOV(){
    return run(
        () -> {
      if(ifInManual){
        angleOfTurretSparkMax.set(0.1);
      }
    });
    
  }

  /**
   * for manual over ride
   * 
   * @return
   */
  public Command leftPOV(){
    return run(
        () -> {
      if(ifInManual){
        angleOfTurretSparkMax.set(-0.1);
      }
    });
    
  }

  public Command centerPOV(){
    return run(
        () -> {
      if(ifInManual){
        resetAngleOfTurret();
      }   
      
    });
    
  }
   public void stopTurret(){
    angleOfTurretSparkMax.set(0); 
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
   * 
   * @param x
   * @param y
   * @return
   */
  public static double floorMod(double x, double y) {
    double result = (x - Math.floor(x / y) * y);
    return result == y ? 0 : result;
  }

  public void turretSpin(double xboxPos){
    if(ifInManual){
      desiredAngleOfTurret = angleOfTurretSparkMax.getEncoder().getPosition() + (4*xboxPos);
    }
  }
  public void flyWheelSpin(double xboxPos){
    if(ifInManual){
      rightFlyWheelSparkMax.set(xboxPos);
    }
  }


  public void switchToManual(){
    ifInManual = true;
  }
    public void switchToAuto(){
    ifInManual = false;  
  }

  public void switchMode(){
    ifInManual = !ifInManual;
  }
}
