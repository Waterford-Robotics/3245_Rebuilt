// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.commands.PathfindToPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

public class RobotContainer {

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(DriveConstants.MaxSpeed * 0.1).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Logging
  private final Telemetry logger = new Telemetry(DriveConstants.MaxSpeed);

  // Xbox Controllers here!
  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);

  // Put subsystems here!
  public static final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  // Auto Chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Real Robot Container!
  public RobotContainer() {

    // Triggers
    configureBindings();

    // Chooser window on SmartDashboard/Shuffleboard/Elastic
    SmartDashboard.putData("AutoMode", m_chooser);

    // Named Command Configuration
    // NamedCommands.registerCommand("Example Command", new ExampleCommand(m_ExampleSubsystem));

    // Autos
    // m_chooser.addOption("Curvy yay", m_robotDrive.getAuto("Curvy yay"));
  }

  // Add button bindings here yep
  private void configureBindings() {

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      m_drivetrain.applyRequest(() ->
        drive.withVelocityX(-m_driverController.getLeftY() * DriveConstants.MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(-m_driverController.getLeftX() * DriveConstants.MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-m_driverController.getRightX() * DriveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
      m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    m_driverController.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
    m_driverController.b().whileTrue(m_drivetrain.applyRequest(() ->
      point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
    ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    m_driverController.back().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
    m_driverController.back().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
    m_driverController.start().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
    m_driverController.start().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    m_driverController.leftBumper().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

    m_drivetrain.registerTelemetry(logger::telemeterize);

    // Zero Gyro - Start Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.kStart)
    .onTrue(
      new InstantCommand(() -> m_drivetrain.resetGyro(), m_drivetrain)
    );

    // TODO: Pathfind to Pose - A
    new JoystickButton(m_driverController.getHID(), ControllerConstants.kA)
      .onTrue(new PathfindToPoseCommand(m_drivetrain, PoseConstants.kRedTrenchLeftAlliance, AutoConstants.kconstraints)
    );
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();   
  } 
}
