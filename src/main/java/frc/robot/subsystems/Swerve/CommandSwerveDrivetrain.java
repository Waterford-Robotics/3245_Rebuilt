package frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import frc.robot.subsystems.Limelight.Localization;
import frc.robot.Constants.VisionConstants;

// It drives the robot!
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    
  // For simulation stuff
  private static final double kSimLoopPeriod = 0.004; // 4 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;

  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

  // Red Alliance sees forward as 180 degrees, Blue Alliance sees as 0 (Limelight)
  public static int AllianceYaw;

  // Keep track if we've ever applied the operator perspective before or not 
  private boolean m_hasAppliedOperatorPerspective = false;

  // Pathplanner config
  RobotConfig config;

  // Field2D for Telemetry Data (Limelight)
  private final Field2d m_field = new Field2d();

  // Swerve request to apply during robot-centric path following
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

  // Swerve requests to apply during SysId characterization 
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

  // SysId routine for characterizing translation. This is used to find PID gains for the drive motors.
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
    new SysIdRoutine.Config(
      null,        // Use default ramp rate (1 V/s)
      Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
      null,        // Use default timeout (10 s)
      // Log state with SignalLogger class
      state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
    ),
    new SysIdRoutine.Mechanism(
      output -> setControl(m_translationCharacterization.withVolts(output)),
      null,
      this
    )
  );

  // SysId routine for characterizing steer. This is used to find PID gains for the steer motors.
  @SuppressWarnings("unused")
  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
    new SysIdRoutine.Config(
      null,        // Use default ramp rate (1 V/s)
      Volts.of(7), // Use dynamic voltage of 7 V
      null,        // Use default timeout (10 s)
      // Log state with SignalLogger class
      state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
    ),
    new SysIdRoutine.Mechanism(
      volts -> setControl(m_steerCharacterization.withVolts(volts)),
      null,
      this
    )
  );

  // SysId routine for characterizing rotation. This is used to find PID gains for the FieldCentricFacingAngle HeadingController. 
  // See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
  @SuppressWarnings("unused")
  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
    new SysIdRoutine.Config(
      // This is in radians per second², but SysId only supports "volts per second"
      Volts.of(Math.PI / 6).per(Second),
      // This is in radians per second, but SysId only supports "volts"
      Volts.of(Math.PI),
      null, // Use default timeout (10 s)
      // Log state with SignalLogger class
      state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
    ),
    new SysIdRoutine.Mechanism(
      output -> {
        // output is actually radians per second, but SysId only supports "volts" 
        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
        // also log the requested output for SysId
        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
      },
      null,
      this
    )
  );

  // The SysId routine to test
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   * 
   * NOTICE: This constructs the underlying hardware devices, so users should not 
   * construct the devices themselves. If they need the devices, they can access them 
   * through getters in the classes.
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
  public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    configureAutoBuilder();
  }

  public CommandSwerveDrivetrain(
    SwerveDrivetrainConstants drivetrainConstants,
    double odometryUpdateFrequency,
    SwerveModuleConstants<?, ?, ?>... modules
  ) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    configureAutoBuilder();
  }

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

    configureAutoBuilder();
  }

  // Configure AutoBuilder
  public void configureAutoBuilder() {
    
    // Load the RobotConfig from the GUI settings.
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
      () -> getState().Pose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      () -> getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> setControl(
        m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
          .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
          .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
      ),
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(10.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(7.0, 0.0, 0.0) // Rotation PID constants
      ),
      config, // The robot configuration
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
      this // Reference to this subsystem to set requirements
    );
  }

  // Returns a command that applies the specified control request to this swerve drivetrain.
  public Command applyRequest(Supplier<SwerveRequest> request) {
    return run(() -> this.setControl(request.get()));
  }

  // Runs the SysId Quasistatic test in the given direction for the routine
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  // Runs the SysId Dynamic test in the given direction for the routine specified by m_sysIdRoutineToApply.
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  // TODO: PUT LOCALIZATION STUFF HERE
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
        DriverStation.getAlliance().ifPresent(allianceColor -> {
          setOperatorPerspectiveForward(
            allianceColor == Alliance.Red
              ? kRedAlliancePerspectiveRotation
              : kBlueAlliancePerspectiveRotation
          );
          m_hasAppliedOperatorPerspective = true;
        }
      );
    }

    updateVisionMeasurements();
  }

  // Updates poseEstimate with the Limelight Readings using MT2 
  public void updateVisionMeasurements() {

    // Setting Yaw to Compensate for Red Alliance Limelight Localization
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        AllianceYaw = 180;
      }
      else if (alliance.get() == DriverStation.Alliance.Blue){
        AllianceYaw = 0;
      }
    }

    // For each limelight...
    for (Localization.LimelightPoseEstimateWrapper estimateWrapper : Localization.getPoseEstimates(getState().Pose.getRotation().getDegrees())) {

      // If there is a tag in view and the pose estimate is valid...
      if (estimateWrapper.tiv && poseEstimateIsValid(estimateWrapper.poseEstimate)) {

        // Add the vision measurement to the swerve drive
        super.addVisionMeasurement(estimateWrapper.poseEstimate.pose,
          estimateWrapper.poseEstimate.timestampSeconds,
          estimateWrapper.getStdvs(estimateWrapper.poseEstimate.avgTagDist));
      }
    }
    
    // Update pos on Field2d
    m_field.setRobotPose(getState().Pose);
    SmartDashboard.putData("Localization/Field", m_field);

    // Localization values
    SmartDashboard.putNumber("Fused Local X", getState().Pose.getX());
    SmartDashboard.putNumber("Fused Local Y", getState().Pose.getY());
  }

  // Check if pose estimate is valid
  private boolean poseEstimateIsValid(LimelightHelpers.PoseEstimate estimate) {
    return estimate != null 
      && Math.abs(getTurnRate()) < VisionConstants.kRejectionRotationRate
      && estimate.avgTagDist < VisionConstants.kRejectionDistance;
  }

  // Pathfinds to pose and avoids obstacles in the way
  public void pathfindToPose(Pose2d targetPose, PathConstraints constraints) {
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
      targetPose, 
      constraints, 
      0.0
    );
    pathfindingCommand.schedule();
  } 

   // Pathfinds to path and then follows that path
   public void pathfindThenFollowPath(String path, PathConstraints constraints) {
    try {
      PathPlannerPath m_path = PathPlannerPath.fromPathFile(path);
      Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
      m_path, 
      constraints
    );
    pathfindingCommand.schedule();
    }
    catch(Exception e) {
      e.printStackTrace();
    }
  } 

  // Simulation stuff
  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    // Run simulation at a faster rate so PID gains behave more reasonably
    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      // use the measured time delta, get battery voltage from WPILib
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  // Turn rate stuff
  public double getTurnRate() {
    return getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
  }

  // Reset Gyro
  public void resetGyro() {
    getPigeon2().setYaw(0);
  }

  // Gets an Auto from PathPlanner
  public Command getAuto(String autoName) {
    return AutoBuilder.buildAuto(autoName);
  }
}
