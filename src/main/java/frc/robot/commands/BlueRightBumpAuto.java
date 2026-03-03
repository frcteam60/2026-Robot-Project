// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueRightBumpAuto extends SequentialCommandGroup {
  /** Creates a new ExampleAuto. */
  public BlueRightBumpAuto(DriveSubsystem driveSubsystem, Turret turret, Feeder chimney, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    ParallelCommandGroup driveAndPickUp = new ParallelCommandGroup(
    new AutoDriveToPose(driveSubsystem, 0.5,  new Pose2d(7.7, 2.3749, null)), 
    new IntakeBalls(intake, 0.7));

    addCommands(
    // Drive backwards for .25 seconds. The driveArcadeAuto command factory
    // intentionally creates a command which does not end which allows us to control
    // the timing using the withTimeout decorator
    new SetPoseAtStartOfAuto(driveSubsystem, new Pose2d(3.69, 2.3749, new Rotation2d(0))),
    new Launch(turret, chimney, intake).withTimeout(6),
    driveAndPickUp,
    new AutoDriveToPose(driveSubsystem, -0.5,  new Pose2d(3.5, 23749., null)),
    new Launch(turret, chimney, intake).withTimeout(10));
    // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
    // total of 10 seconds

    
  }
  public void hi() {
      System.out.println("Hi. Im the Robot");
    }
}
