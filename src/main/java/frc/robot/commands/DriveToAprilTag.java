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
  PIDController xController = new PIDController(0.65, 0, 0);
  PIDController yController = new PIDController(0.7, 0, 0);
  SwerveRequest.RobotCentric drive;
  CommandSwerveDrivetrain drivetrain;
  double setpoint;
  boolean alignY = false;
  double setpointY;
  /** Creates a new DriveToAprilTag. */
  public DriveToAprilTag(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric drive, double setpointX, boolean yAlign, double setpointY) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.drive = drive;
    this.setpoint = setpointX;
    if (yAlign) {
      this.alignY = true;
    }
    this.setpointY = setpointY;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(setpoint);
    if (alignY) {
      yController.setSetpoint(setpointY);
      yController.setTolerance(1);
    }
    xController.setTolerance(1.);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drivetrain.getTV() || drivetrain.getTVFront()) {
    drivetrain.setControl(
      drive
      .withVelocityX(xController.calculate(alignY ? drivetrain.getTYFront() : drivetrain.getTY())*((setpoint < 0) ? -0.3 : 0.3))
      .withRotationalRate(0.)
      .withVelocityY(alignY ? yController.calculate(alignY ? drivetrain.getTXFront() : drivetrain.getTX())*0.3 : 0.)
    );
    System.out.println("Working");
  }
  else {
    drivetrain.setControl(
      drive
      .withVelocityX(0.)
      .withRotationalRate(0)
      .withVelocityY(0)
    );
    System.out.println("Not Working");
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() || yController.atSetpoint();
  }
}
