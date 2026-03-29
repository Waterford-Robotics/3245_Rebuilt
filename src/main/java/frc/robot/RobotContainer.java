// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

public class RobotContainer {

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(DriveConstants.k_maxSpeed * 0.1).withRotationalDeadband(DriveConstants.k_maxAngularRate * 0) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage // Use open-loop control for drive motors
  ); 

  // Xbox Controllers here!
  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.k_driverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(ControllerConstants.k_operatorControllerPort);

  // Put subsystems here!
  public static final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  private final CANRangeSubsystem m_canRangeSubsystem = new CANRangeSubsystem();
  private final ShootSubsystem m_shootSubsystem = new ShootSubsystem();
  private final IndexerSubsystem m_indexSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IntakeFlipoutSubsystem m_flipoutSubsystem = new IntakeFlipoutSubsystem();
  public static final ServoSubsystem m_servoSubsystem1 = new ServoSubsystem(ServoConstants.k_servoID1, 140, 100);
  public static final ServoSubsystem m_servoSubsystem2 = new ServoSubsystem(ServoConstants.k_servoID2, 140, 100);

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

  private SendableChooser<Command> m_action1 = new SendableChooser<>();
  private SendableChooser<Command> m_action2 = new SendableChooser<>();
  private SendableChooser<Command> m_action3 = new SendableChooser<>();
  private SendableChooser<Command> m_action4 = new SendableChooser<>();
  private SendableChooser<Command> m_action5 = new SendableChooser<>();

    public RobotContainer() {

    // Cos those warnings are so annoying (TODO: SILENCE WARNING REMOVE LATER)
    DriverStation.silenceJoystickConnectionWarning(true); 

    // No logs
    SignalLogger.enableAutoLogging(false);

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
      /*
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
      */
      
      m_action1.setDefaultOption("Nothing", Commands.none());
      m_action1.addOption("Shoot Eight", m_autoFactory.Score(2, 1.5));
      
      m_action2.setDefaultOption("Nothing", Commands.none());
      m_action2.addOption("Close/Mid Pose", m_autoFactory.GoOutTrench(PoseConstants.k_redLeftAutoStartingPose, PoseConstants.k_redTrenchLeftNeutralPoseRotated));
      m_action2.addOption("Far Pose", m_autoFactory.GoOutTrench(PoseConstants.k_redLeftAutoStartingPose, PoseConstants.k_redTrenchLeftFarPoseRotated));

      m_action3.setDefaultOption("Nothing", Commands.none());
      m_action3.addOption("Close Path", m_autoFactory.FollowPath("Red Left Trench Close"));
      m_action3.addOption("Mid Path", m_autoFactory.FollowPath("Red Left Trench Mid"));
      m_action3.addOption("Far Path", m_autoFactory.FollowPath("Red Left Trench Far"));

      m_action4.setDefaultOption("Nothing", Commands.none());
      m_action4.addOption("Off Trench Shoot", m_autoFactory.GoBackTrench(PoseConstants.k_redTrenchLeftNeutralPose, PoseConstants.k_redTrenchLeftAlliancePose, PoseConstants.k_redLeftShootPose));

      m_action5.setDefaultOption("Nothing", Commands.none());
      m_action5.addOption("Shoot Hopper", m_autoFactory.Score(0, 4));


      // Puts a chooser on the SmartDashboard!
      SmartDashboard.putData("AutoMode", m_chooser);
      SmartDashboard.putData("Action 1", m_action1);
      SmartDashboard.putData("Action 2", m_action2);
      SmartDashboard.putData("Action 3", m_action3);
      SmartDashboard.putData("Action 4", m_action4);
      SmartDashboard.putData("Action 5", m_action5);
      SmartDashboard.putNumber("Shoot Speed", 0.25);
      SmartDashboard.putNumber("Shoot Angle", 0);

      configureBindings();
    }


  // Config the Button Buttons YAY
  private void configureBindings() {

    /*
     * DRIVER CONTROLLER
     * Left Trig - Intake + Index
     * Right Trig - Index for Shot
     * A - EMERGENCY Assisted Shot Command
     * Start - Zero Gyro
     */

    // Left Trig - Intake
    new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_lefttrig) > 0.05)
    .onTrue(
      new RunCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem).alongWith(
        new AutoIndexCommand(m_indexSubsystem, m_canRangeSubsystem)
      )
    )
    .onFalse(
      new InstantCommand(()-> m_indexSubsystem.stopIndexer(), m_indexSubsystem).alongWith(
        new InstantCommand(() -> m_intakeSubsystem.stop(), m_intakeSubsystem)
      )
    );

    // Right Trig - Index for Shooting
    new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_righttrig) > 0.05)
    .onTrue(
      new RunCommand(() -> m_indexSubsystem.index(), m_indexSubsystem).alongWith(
        new RunCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem)
      )
    )
    .onFalse(
      new InstantCommand(() -> m_intakeSubsystem.stop(), m_intakeSubsystem).alongWith(
        new InstantCommand(() -> m_indexSubsystem.stopIndexer(), m_indexSubsystem)
      )
    );

    // A - Emergency Assisted Shoot Command
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_A)
    .whileTrue(
      new AssistedShootCommand(m_servoSubsystem1, m_servoSubsystem2, m_shootSubsystem)
    );

    // Zero Gyro - Start Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_start)
    .onTrue(
      new InstantCommand(() -> m_drivetrain.resetGyro(), m_drivetrain)
    );

    /*
     * OPERATOR CONTROLLER
     * Right Bump - Reverse Index
     * Left Trig - Rev Up
     * Right Trig - Shoot
     * Y - Reverse Intake
    */    

    // Right Bump - Reverse Index
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_rightbump)
    .onTrue(
      new RunCommand(() -> m_indexSubsystem.reverseIndex(), m_indexSubsystem)
    )
    .onFalse(
      new InstantCommand(() -> m_indexSubsystem.stopIndexer(), m_indexSubsystem)
    );

    // X - Hub Shot
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_X)
    .onTrue(
      new RunCommand(() -> m_shootSubsystem.shoot(ShootConstants.k_hubShotSpeed), m_shootSubsystem)
    )
    .onFalse(
      new InstantCommand(() -> m_shootSubsystem.stopShooter(), m_shootSubsystem)
    );

    // Left Trig - Shoot Rev Up
    new Trigger(() -> m_operatorController.getRawAxis(ControllerConstants.k_lefttrig) > 0.05)
    .whileTrue(
      new AssistedShootCommand(m_servoSubsystem1, m_servoSubsystem2, m_shootSubsystem)
    );

    // Right Trig - Auto Align
    new Trigger(() -> m_operatorController.getRawAxis(ControllerConstants.k_righttrig) > 0.05)
    .onTrue(
      new InstantCommand(() -> m_drivetrain.changeRotationAssistance(), m_drivetrain)
    )
    .onFalse(
      new InstantCommand(() -> m_drivetrain.changeRotationAssistance(), m_drivetrain)
    );

    // Y Operator - Reverse Intake
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_Y)
    .onTrue(
      new RunCommand(() -> m_intakeSubsystem.reverseIntake(), m_intakeSubsystem)
    )
    .onFalse(
      new InstantCommand(() -> m_intakeSubsystem.stop(), m_intakeSubsystem)
    );

    /*****************************
    * TODO: FIGURE INTAKE OUT REMOVE LATER - EXPERIMENTAL 
    **************************/
    new POVButton(m_operatorController.getHID(), ControllerConstants.k_dpadLeft)
    .onTrue(
      new SetIntakeFlipoutCommand(m_flipoutSubsystem, "STOW").andThen(
        new ZeroFlipoutCommand(m_flipoutSubsystem)
      )
    );

    new POVButton(m_operatorController.getHID(), ControllerConstants.k_dpadRight)
    .onTrue(
      new ZeroFlipoutCommand(m_flipoutSubsystem).andThen(
        new SetIntakeFlipoutCommand(m_flipoutSubsystem, "DEPLOY")
      )
    );

    // Back - Zero Flipout
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_back)
    .onTrue(
        new ZeroFlipoutCommand(m_flipoutSubsystem)
      );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
      m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );
  }

  public Command getAutonomousCommand() {
    // return m_chooser.getSelected();
    return new SequentialCommandGroup(
      m_action1.getSelected(),
      m_action2.getSelected(),
      m_action3.getSelected(),
      m_action4.getSelected(),
      m_action5.getSelected()
    );
  }
}
