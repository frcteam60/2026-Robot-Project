// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 2;
    public static final int RIGHT_LEADER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;

    public static final int RIGHT_ENCODER_ID = 0;
    public static final int LEFT_ENCODER_ID = 1;



    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static final class FuelConstants {
    public static final int HOOD_ID = 2;
    public static final double INTAKING_HUNGRY_SPEED = 6.0;
    public static final int HUNGRY_MOTOR_ID = 6;
    public static final int HUNGRY_MOTOR_CURRENT_LIMIT = 40;
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 40;
    // turrets pose
    public static final double centerOfRobotToTurret = 0.187;
    /**
     * in radians
     */
    public static final double angleToTurret = 2.92753197;

    public static final Pose2d RED_HUB = new Pose2d(11.915, 4.035, new Rotation2d(0));
    public static final Pose2d BLUE_HUB = new Pose2d(4.626, 4.035, new Rotation2d(0));


    // Motor controller IDs for Fuel Mechanism motors
    public static final int Feeder_MOTOR_ID = 10;
    public static final int Angle_MOTOR_ID = 3;
    public static final int Left_LAUNCHER_MOTOR_ID = 7;
    public static final int Right_LAUNCHER_MOTOR_ID = 8;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int Angle_MOTOR_CURRENT_LIMIT = 20;
    public static final int Left_MOTOR_CURRENT_LIMIT = 40;
    public static final int Right_MOTOR_CURRENT_LIMIT = 40;


    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 10;
    public static final double LAUNCHING_FEEDER_VOLTAGE = 9;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
    public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    public static final double SPIN_UP_SECONDS = 1;
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int STEERING_WHEEL_PORT = 0;
    public static final int Flight_CONTROLLER_PORT = 1;
    public static final int OPERATOR_CONTROLLER_PORT = 2;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = .7;
    public static final double ROTATION_SCALING = .8;
  }
}
