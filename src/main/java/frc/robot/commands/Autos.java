// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import choreo.Choreo;
import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Autos extends Command {
  AutoFactory autoFactory;
  CommandSwerveDrivetrain drivetrain;
  /** Creates a new Autos. */
  public Autos(CommandSwerveDrivetrain drivetrain) {
     // The drive subsystem
     this.drivetrain = drivetrain;
    autoFactory = this.drivetrain.createAutoFactory();
    // Use addRequirements() here to declare subsystem dependencies.
  }
  /* public static Command firstPathSketch(CommandSwerveDrivetrain drivetrain) {
    PathPlannerPath path0 = new PathPlannerPath.fromChoreoTrajectory("1st_Path_sketch", 1);

    return AutoBuilder.followPath(path0);
    return new RunCommand(
      () -> System.out.println("")
    );
  } */
 public Command firstpathsketch() {
  drivetrain.resetPose(new Pose2d(9.7203264, 4.082779407501221, new Rotation2d(3.141592653589793)));
  return Commands.sequence(
    autoFactory.trajectoryCmd("1st_Path_sketch"));
 }
 public Command testpath() {
  drivetrain.resetPose(new Pose2d(9.66354751586914, 4.0638532638549805, new Rotation2d(Math.PI)));
  return Commands.sequence(
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
