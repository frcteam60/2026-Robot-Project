// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;


public class Intake extends SubsystemBase {
        
   private final SparkMax hungrySparkMax;

                public Intake() {
   
                  
                  hungrySparkMax = new SparkMax(HUNGRY_MOTOR_ID, MotorType.kBrushless);
                SparkMaxConfig hungryConfig = new SparkMaxConfig();
                hungryConfig.smartCurrentLimit(Angle_HUNGRY_MOTOR_CURRENT_LIMIT);
        hungrySparkMax.configure(hungryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("hungry Speed", INTAKING_HUNGRY_SPEED);
  }

  public Command setHungrySpeed(double speed) {
    return run(
        () -> {
        hungrySparkMax.set(speed);
        });
  }

  public void stop() {

  }

  @Override
  public void periodic() {

  }

  public String toString()
  {
  return "I am hungry.\nFeed me FUEL!!!";
  }
}