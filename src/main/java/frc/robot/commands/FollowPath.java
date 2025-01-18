// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.file.Path;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowPath extends Command {
  SwerveRequest.RobotCentric m_drive;
  CommandSwerveDrivetrain m_drivetrain;
  PIDController xController = new PIDController(0.58, 0, 0);
  List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        m_drivetrain.getPoseLL(),
        new Pose2d(6.0, 5.0, Rotation2d.fromDegrees(0))
    );
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null,// The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        null// Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );
  /** Creates a new FollowPath. */
  public FollowPath(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric drive) {
    m_drivetrain = drivetrain;
    m_drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.FollowPathCommand(path);

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
