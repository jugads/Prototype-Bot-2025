// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoint extends Command {
  SwerveRequest.RobotCentric m_drive;
  CommandSwerveDrivetrain m_drivetrain;
  Pose2d m_Point;
  /** Creates a new DriveToPoint. */
  public DriveToPoint(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric drive, Pose2d Point) {
    SwerveRequest.RobotCentric m_drive = drive;
    CommandSwerveDrivetrain m_drivetrain = drivetrain;
    Pose2d m_Point = Point;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.FollowPathCommand(m_drivetrain.Makepath(m_Point));

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
