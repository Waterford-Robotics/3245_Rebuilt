// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

import frc.robot.AutoFactory;

public class RobotContainer {

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(DriveConstants.k_maxSpeed * 0.1).withRotationalDeadband(DriveConstants.k_maxAngularRate * 0) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  // Logging
  // private final Telemetry logger = new Telemetry(DriveConstants.MaxSpeed);

  // Xbox Controllers here!
  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.k_driverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(ControllerConstants.k_operatorControllerPort);


  // Put subsystems here!
  public static final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  private final CANRangeSubsystem m_canRangeSubsystem = new CANRangeSubsystem();
  private final ShootSubsystem m_shootSubsystem = new ShootSubsystem();
  private final IndexerSubsystem m_indexSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public static final ServoSubsystem m_servoSubsystem1 = new ServoSubsystem(ServoConstants.k_servoID1, 140, 100);
  public static final ServoSubsystem m_servoSubsystem2 = new ServoSubsystem(ServoConstants.k_servoID2, 140, 100);
  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();

  // Auto Factory
  private AutoFactory m_autoFactory = new AutoFactory(
    m_drivetrain, 
    m_intakeSubsystem, 
    m_indexSubsystem, 
    m_canRangeSubsystem, 
    m_servoSubsystem2, 
    m_servoSubsystem1, 
    m_shootSubsystem
  );

  // Create New Choosing Option in SmartDashboard for Autos
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {

      // No logs
      SignalLogger.enableAutoLogging(false);

      // Config the bindings for controllers
      configureBindings();

      // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      m_drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(() -> 
          drive.withVelocityX(-m_driverController.getLeftY() * DriveConstants.k_maxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-m_driverController.getLeftX() * DriveConstants.k_maxSpeed) // Drive left with negative X (left)
            .withRotationalRate(m_drivetrain.getAdjustedRotation(-MathUtil.applyDeadband(m_driverController.getRightX(),0.1)) * DriveConstants.k_maxAngularRate) // Drive counterclockwise with negative X (left) (changed **)
        )
      );
      // COMP AUTOS
      m_chooser.addOption("Hub Shot Auto", m_autoFactory.HubShotAuto());
      m_chooser.addOption("Red Left Trench Single Dip", m_autoFactory.RedLeftTrenchSingleDipAuto(PoseConstants.k_redTrenchLeftNeutralPoseRotated, "Red Left Trench"));
      m_chooser.addOption("Red Right Trench Single Dip", m_autoFactory.RedRightTrenchSingleDipAuto(PoseConstants.k_redTrenchLeftNeutralPoseRotated, "Red Left Trench"));
      m_chooser.addOption("Blue Left Trench Single Dip", m_autoFactory.BlueLeftTrenchSingleDipAuto(PoseConstants.k_redTrenchLeftNeutralPoseRotated, "Red Left Trench"));
      m_chooser.addOption("Blue Right Trench Single Dip", m_autoFactory.BlueRightTrenchSingleDipAuto(PoseConstants.k_redTrenchLeftNeutralPoseRotated, "Red Left Trench"));
      m_chooser.addOption("Close Red Left Trench Single Dip", m_autoFactory.RedLeftTrenchSingleDipAuto(PoseConstants.k_redTrenchLeftNeutralPoseRotated, "Red Left Trench Close"));
      m_chooser.addOption("Close Red Right Trench Single Dip", m_autoFactory.RedRightTrenchSingleDipAuto(PoseConstants.k_redTrenchLeftNeutralPoseRotated, "Red Left Trench Close"));
      m_chooser.addOption("Playoffs Left", m_autoFactory.BlueLeftTrenchSingleDipAuto(PoseConstants.k_redTrenchLeftNeutralPoseRotated, "Red Left Trench Close"));
      m_chooser.addOption("Playoffs Right", m_autoFactory.BlueRightTrenchSingleDipAuto(PoseConstants.k_redTrenchLeftNeutralPoseRotated, "Red Left Trench Close"));
      m_chooser.addOption("Far Red Left Trench Single Dip", m_autoFactory.RedLeftTrenchSingleDipAuto(PoseConstants.k_redTrenchLeftFarPoseRotated, "Red Left Trench Far"));
      m_chooser.addOption("Far Red Right Trench Single Dip", m_autoFactory.RedRightTrenchSingleDipAuto(PoseConstants.k_redTrenchLeftFarPoseRotated, "Red Left Trench Far"));
      m_chooser.addOption("Far Blue Left Trench Single Dip", m_autoFactory.BlueLeftTrenchSingleDipAuto(PoseConstants.k_redTrenchLeftFarPoseRotated, "Red Left Trench Far"));
      m_chooser.addOption("Far Blue Right Trench Single Dip", m_autoFactory.BlueRightTrenchSingleDipAuto(PoseConstants.k_redTrenchLeftFarPoseRotated, "Red Left Trench Far"));
      m_chooser.addOption("New: Red Left to Right Trench Single Dip", m_autoFactory.RedLeftTrenchToRightAuto());
      m_chooser.addOption("New: Red Right to Left Trench Single Dip", m_autoFactory.RedRightTrenchToLeftAuto());
      m_chooser.addOption("New: Blue Left to Right Trench Single Dip", m_autoFactory.BlueLeftTrenchToRightAuto());
      m_chooser.addOption("New: Blue Right to Left Trench Single Dip", m_autoFactory.BlueRightTrenchToLeftAuto());

      // Puts a chooser on the SmartDashboard!
      SmartDashboard.putData("AutoMode", m_chooser);
      SmartDashboard.putNumber("Shoot Speed", 0.25);
      SmartDashboard.putNumber("Shoot Angle", 0);
    }

    // Config the Button Buttons YAY
    private void configureBindings() {

    /*
     * DRIVER CONTROLLER
     * 
     * X - Set Servos
     * Y - Intake
     * 
     * Right Bump - Index
     * Right Trig - Shoot
     * Left Trig - Index with CANRange
     */

    // // X - Set Servos
    // new JoystickButton(m_driverController.getHID(), ControllerConstants.k_X)
    // .onTrue(
    //   new InstantCommand(() -> m_servoSubsystem1.setPosition(m_servoSubsystem1.getSetpoint()), m_servoSubsystem1) // Max of 60 for servo
    // )
    // .onTrue(
    //   new InstantCommand(() -> m_servoSubsystem2.setPosition(m_servoSubsystem2.getSetpoint()), m_servoSubsystem2)
    // );

    // Left Trig - Intake
    new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_lefttrig) > 0.05)
    .onTrue(
      new RunCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem)
    )
    .onTrue(
        new AutoIndexCommand(m_indexSubsystem, m_canRangeSubsystem))
    .onFalse(
        new InstantCommand(()-> m_indexSubsystem.stopIndexer(), m_indexSubsystem)
      )
    .onFalse(
      new InstantCommand(() -> m_intakeSubsystem.stop(), m_intakeSubsystem)
    );

    // Right Trig - Index for Shooting
    new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_righttrig) > 0.05)
    .onTrue(
      new RunCommand(() -> m_indexSubsystem.index(), m_indexSubsystem)
    )
    .onTrue(
      new RunCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem)
    )
    .onFalse(
      new InstantCommand(() -> m_intakeSubsystem.stop(), m_intakeSubsystem)
    )
    .onFalse(
      new InstantCommand(() -> m_indexSubsystem.stopIndexer(), m_indexSubsystem)
    );

    // Right Bump - Reverse Index
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_rightbump)
    .onTrue(
      new RunCommand(() -> m_indexSubsystem.reverseIndex(), m_indexSubsystem)
    )
    .onFalse(
      new InstantCommand(() -> m_indexSubsystem.stopIndexer(), m_indexSubsystem)
    );

    // Y Operator - Reverse Intake
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_Y)
    .onTrue(
      new RunCommand(() -> m_intakeSubsystem.reverseIntake(), m_intakeSubsystem)
    )
    .onFalse(
      new InstantCommand(() -> m_intakeSubsystem.stop(), m_intakeSubsystem)
    );

    // Left Bump Operator - Passing
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_leftbump)
    .onTrue(new RunCommand(() -> m_shootSubsystem.shoot(0.40), m_shootSubsystem))
    .onFalse(new InstantCommand(() -> m_shootSubsystem.stopShooter(), m_shootSubsystem));

    // Left Trig - Shoot Rev Up
    new Trigger(() -> m_operatorController.getRawAxis(ControllerConstants.k_lefttrig) > 0.05)
      .whileTrue(
        new ParallelDeadlineGroup(
          new AssistedShootCommand(m_servoSubsystem1, m_servoSubsystem2, m_shootSubsystem), 
          new LEDColorChangeCommand(m_LEDSubsystem, LEDConstants.k_timeBeforeRevComplete)
        )
      )
      .onFalse(
        new InstantCommand(() -> m_LEDSubsystem.setRainbowParty(), m_LEDSubsystem)
      );


    // A Driver - EMERGENCY REV UP
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_A)
    .whileTrue(
        new AssistedShootCommand(m_servoSubsystem1, m_servoSubsystem2, m_shootSubsystem)
      );

    /*
    new Trigger(() -> m_operatorController.getRawAxis(ControllerConstants.k_lefttrig) > 0.05)
      .whileTrue(new RunCommand(() -> m_shootSubsystem.shoot(SmartDashboard.getNumber("Shoot Speed", 0.25))))
      .whileTrue(new RunCommand(() -> m_servoSubsystem1.setPosition(SmartDashboard.getNumber("Shoot Angle", 0))))
      .whileTrue(new RunCommand(() -> m_servoSubsystem2.setPosition(SmartDashboard.getNumber("Shoot Angle", 0))))
      .onFalse(new InstantCommand(() -> m_shootSubsystem.stopShooter()))
      .onFalse(new InstantCommand(() -> m_servoSubsystem1.setPosition(0)))
      .onFalse(new InstantCommand(() -> m_servoSubsystem2.setPosition(0)));
    */

      // .onFalse(
      //   new SetAssistedShotCommand(false)
      // );

    // // Right Trig - Index with CANRange
    // new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_righttrig) > 0.05)
    //   .onTrue(
    //     new AutoIndexCommand(m_indexSubsystem, m_canRangeSubsystem))
    //   .onFalse(
    //     new InstantCommand(()-> m_indexSubsystem.stopIndexer(), m_indexSubsystem)
    //   );

    // Zero Gyro - Start Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_start)
    .onTrue(
      new InstantCommand(() -> m_drivetrain.resetGyro(), m_drivetrain)
    );

    

    // TODO: Pathfind to Pose - A
    //new JoystickButton(m_driverController.getHID(), ControllerConstants.k_A)
    //  .onTrue(new PathfindToPoseCommand(m_drivetrain, PoseConstants.k_redTrenchLeftAlliancePose, AutoConstants.k_constraints)
    //);

    // Switch Between Rotation Assistance - B
    new Trigger(() -> m_operatorController.getRawAxis(ControllerConstants.k_righttrig) > 0.05)
      .onTrue(new InstantCommand(() -> m_drivetrain.changeRotationAssistance(), m_drivetrain))
      .onFalse(new InstantCommand(() -> m_drivetrain.changeRotationAssistance(), m_drivetrain));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
      m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    // m_drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
