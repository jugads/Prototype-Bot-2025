// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KnuckleConstants;

import static frc.robot.Constants.KnuckleConstants.*;
public class Knuckle extends SubsystemBase {
  SparkMax motor;
  public Knuckle() {
    motor = new SparkMax(kMotorID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setKnuckleMotorHigh() {
    motor.set(kHighSpeed);
  }

  public void setKnuckleMotorLow() {
    motor.set(kLowSpeed);
  }

  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  public boolean isAtCurrentThreshold() {
    return getCurrent() > kCurrentThreshold;
  }
  
  public void stopMotor() {
    motor.set(0);
  }
}
