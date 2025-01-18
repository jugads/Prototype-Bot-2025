// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.security.Timestamp;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignWithCoralStation;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.Rotate180;
import frc.robot.commands.RotateToAprilTag;

import com.revrobotics.spark.SparkMax;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.commands.Straighten;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.Straighten;
public class RobotContainer {
    DigitalInput input = new DigitalInput(9);
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final RobotCentric driveRR = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick buttons = new Joystick(1);
    private final SparkMax motor = new SparkMax(3, MotorType.kBrushless);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // private final  SendableChooser<Command> autoChooser;
    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(drivetrain.getKinematics(), new Rotation2d(logger.getCurrentRot()), drivetrain.getModulePositions(), drivetrain.getPoseLL());
    StructPublisher<Pose2d> publisher;
    private AutoChooser autoChooser;
    private AutoFactory autoFactory;
    private final Autos autos = new Autos(drivetrain);
    public RobotContainer() {
        autoChooser = new AutoChooser();
    // Add options to the chooser
    //autoChooser.addRoutine("Example Routine", this::exampleRoutine);
    autoChooser.addCmd("firstpathsketch", () -> autos.testpath());
    
    // Put the auto chooser on the dashboard
    SmartDashboard.putData(autoChooser);
    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autos.testpath());
       
        // SmartDashboard.putNumber("Current Draw Climber", motor.getOutputCurrent());
        publisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose2d.struct).publish();
        configureBindings();
        if (drivetrain.getTVFront()) {
            poseEstimator.resetPose(drivetrain.getFrontLLPose());
        }
    }
    public void getInput() {
        // if (drivetrain.getTV()) {
        // poseEstimator.addVisionMeasurement(drivetrain.getPoseLL(), Utils.getCurrentTimeSeconds());
        // poseEstimator.update(drivetrain.getPoseLL().getRotation(), drivetrain.getModulePositions());
        // }
        if (drivetrain.getTVFront()) {
        poseEstimator.addVisionMeasurement(drivetrain.getFrontLLPose(), Utils.getCurrentTimeSeconds());
        poseEstimator.update(drivetrain.getFrontLLPose().getRotation(), drivetrain.getModulePositions());
        }
        // else if (drivetrain.getTVRear()) {
        // poseEstimator.addVisionMeasurement(drivetrain.getRearLLPose(), Utils.getCurrentTimeSeconds());
        // poseEstimator.update(drivetrain.getRearLLPose().getRotation(), drivetrain.getModulePositions());
        // }
        else {
        poseEstimator.update(drivetrain.getPigeon2().getRotation2d(), drivetrain.getModulePositions());
        }
        
        
        
        publisher.set(poseEstimator.getEstimatedPosition());
    }

    private void configureBindings() {
        System.out.println(input.get());
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(
                () ->
                drive
                .withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(
                () -> 
                driveRR
                .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        joystick.start().onTrue(
          new InstantCommand(
            () -> poseEstimator.resetPose(
                drivetrain.getTVFront() ?
                drivetrain.getFrontLLPose() :
                (
                    drivetrain.getTV() ?
                    drivetrain.getPoseLL() :
                    drivetrain.getRearLLPose()
                )
            )
          )  
        );
        joystick.povUp().whileTrue(
            new RunCommand(
              () -> motor.set(0.15)
            )
          );
          joystick.povDown().whileTrue(
            new RunCommand(
              () -> motor.set(-0.15)
            )
          );
        joystick.povUp().whileFalse(
            new RunCommand(
                () -> motor.set(0.)
            )
        );
        joystick.povDown().whileFalse(
            new RunCommand(
                () -> motor.set(0.)
            )
        );
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    
        new JoystickButton(buttons, 12).whileTrue(
            new SequentialCommandGroup(
                new RotateToAprilTag(drivetrain, driveRR, 3),
                new DriveToAprilTag(drivetrain, driveRR, 20, false, 0.),
                new RotateToAprilTag(drivetrain, driveRR, 2)
            )
        );
        joystick.a().whileTrue(
            new DriveToAprilTag(drivetrain, driveRR, -20, true, -9)
        );
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // public Command getAutonomousCommand() {

    //     return 
    // }
}
