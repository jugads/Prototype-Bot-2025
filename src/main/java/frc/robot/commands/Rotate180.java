// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Rotate180 extends Command {
  /** Creates a new Rotate180. */
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric drive;
  Telemetry telemetry;
  PIDController controller = new PIDController(0.119, 0, 0.0);
  public Rotate180(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric drive, Telemetry telemetry) {
    this.drivetrain = drivetrain;
    this.drive = drive;
    this.telemetry = telemetry;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(telemetry.getCurrentRot() + 180);
    SmartDashboard.putNumber("Setpoint", controller.getSetpoint());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    telemetry.telemeterize(drivetrain.getState());
    if (!controller.atSetpoint()) {
    drivetrain.setControl(drive
    .withVelocityX(0)
    .withVelocityY(0)
    .withRotationalRate(controller.calculate(telemetry.getCurrentRot())));
    SmartDashboard.putNumber("Rot", telemetry.getCurrentRot());

    }
    else {
      drivetrain.setControl(drive
    .withVelocityX(0)
    .withVelocityY(0)
    .withRotationalRate(0)
    );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
