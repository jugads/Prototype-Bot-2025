// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToAprilTag extends Command {
  /** Creates a new RotateToAprilTag. */
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric drive;
  double tolerance;
  PIDController controller = new PIDController(0.11875, 0, 0.007);
  public RotateToAprilTag(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric drive, double tolerance) {
    this.drivetrain = drivetrain;
    this.drive = drive;
    this.tolerance = tolerance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      controller.setSetpoint(0);
      controller.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setControl(
    drive
    .withRotationalRate(controller.calculate(drivetrain.getTX()))
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drivetrain.getTX()) <= 3;
  }
}
