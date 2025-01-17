// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import static frc.robot.Constants.AlgaeScorerConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaeScorer extends SubsystemBase {
  /** Creates a new AlgaeScorer. */
  SparkMax motor;
  public AlgaeScorer() {
    motor = new SparkMax(kMotorID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void runAlgaeScorer(double speed) {
    motor.set(speed);
  }
  public void stopMotor() {
    motor.set(0.);
  }
  public double getAlgaeScorerCurrent() {
    return motor.getOutputCurrent();
  }
  public boolean algaeScorerAtThreshold() {
    return getAlgaeScorerCurrent() < kCurrentThreshold;
  }
}
