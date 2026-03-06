// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.commands.PathfindToPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.Constants.ServoConstants;
import frc.robot.commands.AssistedShootCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.subsystems.CANRangeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ServoSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class RobotContainer {

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(DriveConstants.MaxSpeed * 0.1).withRotationalDeadband(DriveConstants.MaxAngularRate * 0) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Logging
  private final Telemetry logger = new Telemetry(DriveConstants.MaxSpeed);

  // Xbox Controllers here!
  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);

  // Put subsystems here!
  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final CANRangeSubsystem m_canRangeSubsystem = new CANRangeSubsystem();
  private final ShootSubsystem m_shootSubsystem = new ShootSubsystem();
  private final IndexerSubsystem m_indexSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public static final ServoSubsystem m_servoSubsystem1 = new ServoSubsystem(ServoConstants.k_servoID1, 140, 100);
  public static final ServoSubsystem m_servoSubsystem2 = new ServoSubsystem(ServoConstants.k_servoID2, 140, 100);

    public RobotContainer() {
      SignalLogger.enableAutoLogging(false);
      configureBindings();
    }

    // Config the Button Buttons YAY
    private void configureBindings() {

    /*
     * DRIVER CONTROLLER
     * 
     * X - Set Servoes
     * Y - Intake
     * 
     * Right Bump - Index
     * Right Trig - Shoot
     * Left Trig - Index with CANRange
     */
    /*
    // X - Set Servos
    new JoystickButton(m_driverController.getHID(), ControllerConstants.kX)
    .onTrue(
      new InstantCommand(() -> m_servoSubsystem1.setPosition(m_servoSubsystem1.getSetpoint()), m_servoSubsystem1) // Max of 60 for servo
    )
    .onTrue(
      new InstantCommand(() -> m_servoSubsystem2.setPosition(m_servoSubsystem2.getSetpoint()), m_servoSubsystem2)
    );

    // Y - Intake
    new JoystickButton(m_driverController.getHID(), ControllerConstants.kY)
    .whileTrue(
      new RunCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem)
    )
    .onFalse(
      new InstantCommand(() -> m_intakeSubsystem.stop(), m_intakeSubsystem)
    );

    // Right Bump - Index
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_rightbump)
    .onTrue(
      new RunCommand(() -> m_indexSubsystem.index(), m_indexSubsystem)
    )
    .onFalse(
      new InstantCommand(() -> m_indexSubsystem.stopIndexer(), m_indexSubsystem)
    );

    // Left Trig - Shoot
    new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_lefttrig) > 0.05)
      .whileTrue(
        new AssistedShootCommand(drivetrain, m_servoSubsystem1, m_servoSubsystem2, m_shootSubsystem))
      .onFalse(
        new InstantCommand(()-> m_shootSubsystem.stopShooter(), m_shootSubsystem)
      )
      .onFalse(
        new InstantCommand(() -> m_servoSubsystem1.setPosition(0), m_servoSubsystem1)
      )
      .onFalse(
        new InstantCommand(() -> m_servoSubsystem2.setPosition(0), m_servoSubsystem2)
      );

    // Right Trig - Index with CANRange
    new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_righttrig) > 0.05)
      .whileTrue(
        new IndexCommand(m_indexSubsystem, m_canRangeSubsystem))
      .onFalse(
        new InstantCommand(()-> m_indexSubsystem.stopIndexer(), m_indexSubsystem)
      );

    // Zero Gyro - Start Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.kStart)
    .onTrue(
      new InstantCommand(() -> drivetrain.resetGyro(), drivetrain)
    );

    // TODO: Pathfind to Pose - A
    new JoystickButton(m_driverController.getHID(), ControllerConstants.kA)
      .onTrue(new PathfindToPoseCommand(drivetrain, PoseConstants.kRedTrenchLeftAlliance, AutoConstants.kconstraints)
    );

    // Switch Between Rotation Assistance - B
    new JoystickButton(m_driverController.getHID(), ControllerConstants.kB)
    .onTrue(
      new InstantCommand(() -> drivetrain.changeRotationAssistance(), drivetrain)
    );
    */

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> 
        drive.withVelocityX(-m_driverController.getLeftY() * DriveConstants.MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(-m_driverController.getLeftX() * DriveConstants.MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(drivetrain.getAdjustedRotation(-MathUtil.applyDeadband(m_driverController.getRightX(),0.1)) * DriveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left) (changed **)
      )
    );

    /*

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
      drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    // m_driverController.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
    m_driverController.b().whileTrue(m_drivetrain.applyRequest(() ->
      point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
    ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    m_driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    //m_drivetrain.registerTelemetry(logger::telemeterize);
    */
  }

  public Command getAutonomousCommand() {
    // Simple drive forward auton
    final var idle = new SwerveRequest.Idle();
    return Commands.sequence(
      // Reset our field centric heading to match the robot
      // facing away from our alliance station wall (0 deg).
      drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
      // Then slowly drive forward (away from us) for 5 seconds.
      drivetrain.applyRequest(() ->
        drive.withVelocityX(0.5)
          .withVelocityY(0)
          .withRotationalRate(0)
      )
      .withTimeout(5.0),
      // Finally idle for the rest of auton
      drivetrain.applyRequest(() -> idle)
    );
  }
}
