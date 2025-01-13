// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Telemetry;
import static frc.robot.Constants.DrivetrainConstants.*;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Straighten extends Command {
  /** Creates a new Straighten. */
  private final CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.FieldCentric drive;
    SwerveRequest.FieldCentric updatedDrive;
    private final PIDController controller = new PIDController(0.025, 0., 0.004);
    private final PIDController controller1 = new PIDController(0.045, 0., 0.004);
    private final Telemetry telemetry;
  public Straighten(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = drivetrain;
        this.drive = drive;
        this.telemetry = new Telemetry(kMaxSpeed);

        addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(0.);
    controller.setTolerance(0.1);
    m_drivetrain.registerTelemetry(telemetry::telemeterize);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    telemetry.telemeterize(m_drivetrain.getState());
        if (telemetry.getCurrentRot() <= 45) { // Ensure telemetry is up to date
        updatedDrive = drive
            .withVelocityX(0.)
            .withVelocityY(0.)
            .withRotationalRate(kMaxSpeed * controller1.calculate(telemetry.getCurrentRot()));
        }
        if (telemetry.getCurrentRot() > 45) { // Ensure telemetry is up to date
        updatedDrive = drive
            .withVelocityX(0.)
            .withVelocityY(0.)
            .withRotationalRate(kMaxSpeed * controller.calculate(telemetry.getCurrentRot()));
        }
        m_drivetrain.setControl(updatedDrive);
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
