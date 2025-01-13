// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ChuteConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chute extends SubsystemBase {
  SparkMax motor;
  public Chute() {
    motor = new SparkMax(kMotorID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {

  }
  
  public void runMotor(double speed) {
    motor.set(speed);
  }

  public void stopMotor() {
    motor.set(0);
  }
}
