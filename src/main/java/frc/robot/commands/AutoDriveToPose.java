// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDriveToPose extends Command {
  
  DriveSubsystem driveSubsystem;
  Pose2d pose;
  double speed;
  double rampDown;
  boolean ifFinished = false;

  PIDController robotYawController;

  public AutoDriveToPose(DriveSubsystem driveSystem, double speed, Pose2d poseToDriveTo) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    pose = poseToDriveTo;
    this.speed = speed;
    robotYawController = new PIDController(0.1, 0, 0);
    robotYawController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Setting the values here instead of in initialize feeds the watchdog on the
  // arcade drive object
  @Override
  public void execute() {
    rampDown = Math.sqrt(constrain(1, 0, Math.abs(getDistanceBetween(pose, driveSubsystem.getRobotPose()))));


    if(Math.abs(getDistanceBetween(pose, driveSubsystem.getRobotPose())) < 0.3){
      ifFinished = true;
    }

    driveSubsystem.driveArcade(speed*rampDown, robotYawController.calculate(driveSubsystem.getRobotPose().getRotation().getRadians(), 
                                                  Math.atan2(pose.getY() - driveSubsystem.getRobotPose().getY(), pose.getX() - driveSubsystem.getRobotPose().getX())));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ifFinished;
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
}
