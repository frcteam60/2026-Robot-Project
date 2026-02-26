// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OperatorConstants.*;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.LaunchSequence;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final Turret shooter = new Turret();
  private final Intake HungryIntake = new Intake();
  private final Feeder chimney = new Feeder();
  //the steering mechanism
  private final Joystick steeringWheeeeel = new Joystick(STEERING_WHEEL_PORT);
  //the acceleration joystick
  private final CommandJoystick vroomVroomStick = new CommandJoystick(Flight_CONTROLLER_PORT);
  
  // The driver's controller
  // private final CommandXboxController driverController = new CommandXboxController(
  //     0);

  // the shooting controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
 
 public RobotContainer() {
    configureBindings();
    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    autoChooser.setDefaultOption("Autonomous", new ExampleAuto(driveSubsystem, shooter, chimney, HungryIntake));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // While the left bumper on operator controller is held, intake Fuel

    //operatorController.leftBumper().whileTrue(new Intake());

    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    //operatorController.rightBumper().whileTrue(new LaunchSequence(shooter));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    //operatorController.a().whileTrue(new Eject(shooter));

    //operatorController.leftBumper().whileTrue(new IntakeBalls(HungryIntake, 0.5));

    
    operatorController.b().whileTrue(HungryIntake.setHungryRpm(-700));
    operatorController.b().onFalse(HungryIntake.setHungrySpeed(0.0));
    //operatorController.b().whileTrue(new IntakeBalls(HungryIntake, -0.85));

    operatorController.leftTrigger().whileTrue(shooter.shoot(chimney, HungryIntake));
    operatorController.leftTrigger().onFalse(HungryIntake.setHungrySpeed(0));
    operatorController.leftTrigger().onFalse(chimney.setSpeedCommand(0));

    //operatorController.rightTrigger().whileTrue(HungryIntake.setHungryRpm(1000));
     operatorController.rightTrigger().whileTrue(HungryIntake.setHungryRpm(5000));
    operatorController.rightTrigger().onFalse(HungryIntake.setHungrySpeed(0));


    operatorController.leftBumper().whileTrue(chimney.setSpeedCommand(-1));
    operatorController.leftBumper().onFalse(chimney.setSpeedCommand(0));
    
    

    operatorController.x().onTrue(Commands.runOnce(shooter::switchMode));
    //operatorController.x().whileFalse(Commands.runOnce(shooter::switchToAuto));

    operatorController.start().onTrue(shooter.centerPOV());
    operatorController.povCenter().onFalse(Commands.runOnce(shooter::stopTurret));

    operatorController.povLeft().whileTrue(shooter.leftPOV());
    operatorController.povLeft().onFalse(Commands.runOnce(shooter::stopTurret));

    operatorController.povRight().whileTrue(shooter.rightPOV());
    operatorController.povRight().onFalse(Commands.runOnce(shooter::stopTurret));




    vroomVroomStick.povUp().whileTrue(driveSubsystem.turnToAngle(0));

    vroomVroomStick.povRight().whileTrue(driveSubsystem.turnToAngle(90));

    vroomVroomStick.povDown().whileTrue(driveSubsystem.turnToAngle(180));

    vroomVroomStick.povLeft().whileTrue(driveSubsystem.turnToAngle(270));


    vroomVroomStick.button(1).onTrue(Commands.runOnce(driveSubsystem::shiftToHigh));
    vroomVroomStick.button(2).onTrue(Commands.runOnce(driveSubsystem::shiftToLow));
    
    //driverController.rightBumper().onTrue(Commands.runOnce(driveSubsystem::shiftGears));
    
    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)

    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, vroomVroomStick, steeringWheeeeel));

    shooter.setDefaultCommand(shooter.run(() -> shooter.stop()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();

  }

  public void updateRobotPose(){
    shooter.updateRobotPose(driveSubsystem.getRobotPose());
  }

  /**
   * MUST BE CALLED IN teleopPeriodic in Robot
   */
  public void teleopPeriodic(){
    //driveSubsystem.autoShiftGears();
    //shooter.trackTarget();
    shooter.flyWheelSpin(operatorController.getRightY());
    // the hood is not working
    //shooter.joyStickServo(operatorController.getRightX());
    shooter.turretSpin(operatorController.getLeftX());

  }

  /**
   * start timer
   */
  public void startTimer(){
    shooter.setUpTimer();
  }


  public void autoMouseInnit(){
    shooter.findAllianceColor();

  }

  public void teleopInit(){
    shooter.findAllianceColor();
  }

  public void setTurretPos(){
    shooter.resetAngleOfTurret();
  }  
  
}
