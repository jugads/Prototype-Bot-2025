package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private Field2d field = new Field2d();
    private double m_lastSimTime;
    NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTable m_limelightRear = NetworkTableInstance.getDefault().getTable("limelight-back");
    NetworkTable m_limelightFront = NetworkTableInstance.getDefault().getTable("limelight-front");
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    // private DigitalInput sensor = new DigitalInput(9);
    NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    NetworkTable table = ntInstance.getTable("Pose");
    NetworkTable poseTable = ntInstance.getTable("MyPose");
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(getKinematics(), getPigeon2().getRotation2d(), getModulePositions(), getFrontLLPose());
    private final SwerveRequest.ApplyRobotSpeeds m_ApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final PIDController m_pathXController = new PIDController(10, 0, 0);
    private final PIDController m_pathYController = new PIDController(10, 0, 0);
    private final PIDController m_pathThetaController = new PIDController(7, 0, 0);
    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null        // Use default timeout (10 s)
            // Log state with SignalLogger class
            // state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null        // Use default timeout (10 s)
            // Log state with SignalLogger class
            // state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null // Use default timeout (10 s)
            // Log state with SignalLogger class
            // state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                // SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        RobotConfig config;

        try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }

    }
        //configureAutoBuilder();
        

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        //configureAutoBuilder();
    }


    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        //configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(()-> getPose(), 
                                this::resetPose,
                                () -> getState().Speeds, 
                                (speeds, feedforwards) -> setControl(
                                    m_ApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                                ), 
                                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                                new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ), 
                                config, 
                                () -> {
                                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                                    // This will flip the path being followed to the red side of the field.
                                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                      
                                    var alliance = DriverStation.getAlliance();
                                    if (alliance.isPresent()) {
                                      return alliance.get() == DriverStation.Alliance.Red;
                                    }
                                    return false;
                                  },
                                    this);

                                    
                }
                catch (Exception ex) {
                    DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
                }
        }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    
    /* private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    } */
    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        poseEstimator.update(getPigeon2().getRotation2d(), getModulePositions());
        if (getTVFront()) {
            poseEstimator.resetPose(new Pose2d(getFrontLLPose().getTranslation(), getPigeon2().getRotation2d()));
        }
        else if (getTV()) {
            poseEstimator.resetPose(new Pose2d(getPoseLL().getTranslation(), getPigeon2().getRotation2d()));
        }
        if (getFrontLLPose().getRotation().getDegrees() - getPose().getRotation().getDegrees() > 90) {
            getPigeon2().setYaw(getFrontLLPose().getRotation().getDegrees());
            
        }
        // SmartDashboard.putBoolean("Sensor Val", sensor.get());
        //  * Periodically try to apply the operator perspective.
        //  * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        //  * This allows us to correct the perspective in case the robot code restarts mid-match.
        //  * Otherwise, only check and apply the operator perspective if the DS is disabled.
        //  * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
        //  */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
            () -> this.getState().Pose,
            this::resetPose,
            this::followPath,
            false,
            this,
            trajLogger
        );
    }
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    public void followPath(SwerveSample sample) {
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getFrontLLPose();

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        setControl(
            m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }
    public double getTX() {
        return m_limelight.getEntry("tx").getDouble(0.0);
      }
      public Pose2d getPoseNT() {
        return (Pose2d) NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).getEntry(getFrontLLPose(), (PubSubOption)null);
      }
      public double getTY() {
        return m_limelight.getEntry("ty").getDouble(0.0);
      }
      public double getTXFront() {
        return m_limelightFront.getEntry("tx").getDouble(0.);
      }
      public double getTYFront() {
        return m_limelightFront.getEntry("ty").getDouble(0.);
      }
      public boolean getTVFront() {
        return m_limelightFront.getEntry("tv").getDouble(0.0) == 1.0;
      }
      public boolean getTV() {
        return m_limelight.getEntry("tv").getDouble(0.0) == 1.0;
      }
      public boolean getTVRear() {
        return m_limelightRear.getEntry("tv").getDouble(0.0) == 1.0;
      }
      public double getTZ() {
        return m_limelight.getEntry("ty").getDouble(0.0);
      }
      public Pose2d getPoseLL() {
        var array = m_limelight.getEntry("botpose_wpired").getDoubleArray(new double[]{});
        double[] result = {array[0], array[1], array[5]};
        Pose2d pose = new Pose2d(result[0], result[1], new Rotation2d(result[2]));
        return pose;
        // double[] poseArray = {pose.getX(), pose.getY(), ((pose.getRotation().getDegrees())/360)+(pose.getRotation().getDegrees()%360)};
        // table.getEntry("RobotPose").setDoubleArray(poseArray);
        // SmartDashboard.putNumberArray("Raw Pose", result);
      }
      public Pose2d getRearLLPose() {
        var array = m_limelightRear.getEntry("botpose_wpired").getDoubleArray(new double[]{});
        double[] result = {array[0], array[1], array[5]};
        Pose2d pose = new Pose2d(result[0], result[1], new Rotation2d(result[2]));
        return pose;
      }
      public Pose2d getFrontLLPose() {
        var array = m_limelightFront.getEntry("botpose_wpired").getDoubleArray(new double[]{});
        double[] result = {array[0], array[1], array[5]};
        Pose2d pose = new Pose2d(result[0], result[1], new Rotation2d(result[2]));
        return pose;
      }
      public SwerveModulePosition[] getModulePositions() {
        return getState().ModulePositions;
      }

      public PathPlannerPath Makepath(Pose2d Point) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        getPose(),
        Point
    );
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null,// The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        null// Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );
    return path;
      }
      public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
      }
      public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
      }
      public void resetGyro() {
        getPigeon2().reset();
      }

      public Command FollowPathCommand(PathPlannerPath path) {
        configureAutoBuilder();

        //PathConstraints constraints = new PathConstraints(
        //3.0, 4.0,
        //Units.degreesToRadians(540), Units.degreesToRadians(720));
        //Command pathfindnCommand = AutoBuilder.pathfindThenFollowPath(path, null);

        //Pose2d targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));
        // Command pathfindingCommand = AutoBuilder.pathfindToPose(
        // targetPose,
        // constraints,
        // 0.0, // Goal end velocity in meters/sec
        // 0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        // );


        return AutoBuilder.followPath(path); }
}
