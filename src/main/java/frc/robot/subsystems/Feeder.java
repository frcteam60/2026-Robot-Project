// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;


public class Feeder extends SubsystemBase {
  private final SparkMax feederSparkMax;

  /** Creates a new CANBallSubsystem. */
  public Feeder() {
    // create brushed motors for each of the motors on the launcher mechanism
    
    feederSparkMax = new SparkMax(Feeder_MOTOR_ID, MotorType.kBrushless);

    // create the configuration for the angle motor, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederSparkMax.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    //SmartDashboard.putNumber("Feeder Speed", INTAKING_FEEDER_VOLTAGE);
  }

  // A method to set the voltage of the intake roller
  public void setFeedSpeed(double speed) {
    feederSparkMax.set(speed);
  }

  public Command setSpeedCommand(double speed) {
    return runOnce(
        () -> {
        feederSparkMax.set(speed);
      });
  }
}
