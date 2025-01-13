// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToAprilTag extends Command {
  PIDController xController = new PIDController(0.3, 0, 0);
  PIDController yController = new PIDController(0.3, 0, 0);
  SwerveRequest.RobotCentric drive;
  CommandSwerveDrivetrain drivetrain;
  /** Creates a new DriveToAprilTag. */
  public DriveToAprilTag(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.drive = drive;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (drivetrain.getTV()) {
      xController.setSetpoint(25);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setControl(
      drive
      .withVelocityX(xController.calculate(drivetrain.getTY()))
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
