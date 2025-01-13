// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class Arm extends SubsystemBase {
  SparkMax motor = new SparkMax(19, MotorType.kBrushless);
  // SparkMax motor1 = new SparkMax(3, MotorType.kBrushless);
  SparkAbsoluteEncoder encoder = motor.getAbsoluteEncoder();
  /** Creates a new Arm. */
  public Arm() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void run(double speed) {
    motor.set(speed);
    // motor1.set(speed);
  }
  public double getEncoderVal() {
    return encoder.getPosition();
  }
}
