// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Joystick;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  /** Creates a new Drive. */
  DriveSubsystem driveSubsystem;
  CommandJoystick vroomVroomStick;
  Joystick steeringWheel;
  double turning;
  double turnValue;
  double drive;

  public Drive(DriveSubsystem driveSystem, CommandJoystick vroomVroomStick, Joystick stearingWheeeeel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    this.vroomVroomStick = vroomVroomStick;
    steeringWheel = stearingWheeeeel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // The Y axis of the controller is inverted so that pushing the
  // stick away from you (a negative value) drives the robot forwards (a positive
  // value). The X axis is scaled down so the rotation is more easily
  // controllable.
  @Override
  public void execute() {
    turning = MathUtil.applyDeadband(steeringWheel.getX(), 0.07);
    turnValue = constrain(1, -1, (Math.abs(turning)/turning)*Math.sqrt(Math.abs(6.5*turning)));
    drive = -constrain(1, -1, 2*vroomVroomStick.getY());
    System.out.println("Drive value" + drive);
    System.out.println("Turning" + turning);


    // driveSubsystem.driveArcade(-constrain(1, -1, 2*vroomVroomStick.getY()), 
    //         constrain(1, -1, Math.abs(turning)/turning)*Math.sqrt(Math.abs(2.5*turning)));

    driveSubsystem.driveArcade(drive, turning);


  

    //driveSubsystem.fieldCentricDrive(MathUtil.applyDeadband(-controller.getRightX(), 0.05), MathUtil.applyDeadband(-controller.getRightY(), 0.05), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
    //driveSubsystem.fieldCentricDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
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
