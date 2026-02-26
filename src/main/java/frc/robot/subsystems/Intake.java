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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;


public class Intake extends SubsystemBase {
   private final SparkMax hungrySparkMax;
   private final PIDController intakController;

  public Intake() {
      intakController = new PIDController(0.0003, 0, 0.00003);
      //intakController = new PIDController(0, 0, 0);
      hungrySparkMax = new SparkMax(HUNGRY_MOTOR_ID, MotorType.kBrushless);
      SparkMaxConfig hungryConfig = new SparkMaxConfig();
      hungryConfig.smartCurrentLimit(HUNGRY_MOTOR_CURRENT_LIMIT);
      hungrySparkMax.configure(hungryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /**
   * 1 to -1
   * 
   * @param speed
   * @return
   */
  public Command setHungrySpeed(double speed) {
    return runOnce(
        () -> {
        hungrySparkMax.set(speed);
      });
  }

  /**
   * rpm
   * 
   * @param speed
   * @return
   */
  public Command setHungryRpm(double speed) {
    return run(
        () -> {
        hungrySparkMax.set(intakController.calculate(hungrySparkMax.getEncoder().getVelocity(), speed));
      });
  }

  public void stop() {
    hungrySparkMax.set(0); 
  }

  @Override
  public void periodic() {

  }

  public String toString()
  {
  return "I am hungry.\nFeed me FUEL!!!";
  }
}