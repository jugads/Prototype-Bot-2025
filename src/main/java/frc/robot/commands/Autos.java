// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;

import choreo.Choreo;
import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Autos extends Command {
  AutoFactory autoFactory;
  CommandSwerveDrivetrain drivetrain;
  AutoTrajectory traj;
  /** Creates a new Autos. */
  public Autos(CommandSwerveDrivetrain drivetrain) {
     // The drive subsystem
     this.drivetrain = drivetrain;
    autoFactory = this.drivetrain.createAutoFactory();

    // Use addRequirements() here to declare subsystem dependencies.
  }

 public Command testpath() {
  return Commands.sequence(
    new SequentialCommandGroup(
      new InstantCommand(() -> drivetrain.resetGyro(0)),
      new InstantCommand(() -> drivetrain.resetPose(new Pose2d(9.66354751586914, 4.0638532638549805, drivetrain.getPigeon2().getRotation2d())))
    ),
    autoFactory.trajectoryCmd("New Path"));
 }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
