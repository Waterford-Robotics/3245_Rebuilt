// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();
  private final ShootSubsystem m_shootSubsystem = new ShootSubsystem();
  private final IndexerSubsystem m_indexSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IntakeFlipoutSubsystem m_flipoutSubsystem = new IntakeFlipoutSubsystem();
  public static final ServoSubsystem m_servoSubsystem1 = new ServoSubsystem(ServoConstants.k_servoID1, 140, 100);
  public static final ServoSubsystem m_servoSubsystem2 = new ServoSubsystem(ServoConstants.k_servoID2, 140, 100);

  // Create New Choosing Option in SmartDashboard for Autos
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

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

    // NAMED COMMANDS
    NamedCommands.registerCommand("Stow Flipout", new SetIntakeFlipoutCommand(m_flipoutSubsystem, "STOW"));
    NamedCommands.registerCommand("Deploy Flipout", DeployflipoutCommandGroup());
    NamedCommands.registerCommand("Auto Intakedex", AutoIntakedexCommandGroup());
    NamedCommands.registerCommand("Run Intake", new InstantCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem));
    NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> m_intakeSubsystem.stop(), m_intakeSubsystem));
    NamedCommands.registerCommand("Auto Index", new AutoIndexCommand(m_indexSubsystem, m_canRangeSubsystem));
    NamedCommands.registerCommand("Index", new IndexForSecsCommand(m_indexSubsystem, 5));
    NamedCommands.registerCommand("Distance Shot", new AssistedShootForSecsCommand(m_servoSubsystem1, m_servoSubsystem2, m_shootSubsystem, 5));

    // COMP AUTOS
    // m_chooser.addOption("Left Trench Snipe Double Dip Auto", m_drivetrain.getAuto("Left Trench Snipe Double Dip Auto"));  
    m_chooser.addOption("Left Trench SWIPE Double Dip Auto", m_drivetrain.getAuto("Left Trench Swipe Double Dip Auto"));

    configureBindings();
    SmartDashboard.putData("Auto Chooser", m_chooser);
  }


  // Config the Button Buttons YAY
  private void configureBindings() {

    /*
     * DRIVER CONTROLLER
     * Left Trig - Intake + Index
     * Right Trig - Index for Shot
     * A - EMERGENCY Assisted Shot Command
     * Start - Zero Gyro
     * X - stow
     * Y - deploy
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

    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_X)
    .onTrue(
      new SequentialCommandGroup(
        new SetIntakeFlipoutCommand(m_flipoutSubsystem, "STOW"),
        new ZeroFlipoutCommand(m_flipoutSubsystem, IntakeConstants.k_intakeRetractedAngle)
      )
    );

    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_Y)
    .onTrue(
      new SequentialCommandGroup(
        new SetIntakeFlipoutCommand(m_flipoutSubsystem, "DEPLOY"),
        new ZeroFlipoutCommand(m_flipoutSubsystem, IntakeConstants.k_intakeDeployedAngle)
      )
    );


    // Testing
    /*

    // X - Shot
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_X)
    .onTrue(
      new RunCommand(() -> m_shootSubsystem.shoot(), m_shootSubsystem)
    )
    .onFalse(
      new InstantCommand(() -> m_shootSubsystem.stopShooter(), m_shootSubsystem)
    );

    // Y - Servos
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_Y)
    .onTrue(
      new RunCommand(() -> m_servoSubsystem1.setPosition(m_servoSubsystem1.getSetpoint()), m_servoSubsystem1)
    )
    .onTrue(
      new RunCommand(() -> m_servoSubsystem2.setPosition(m_servoSubsystem2.getSetpoint()), m_servoSubsystem2)
    );

    // B - Auto Align
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_B)
    .onTrue(
      new InstantCommand(() -> m_drivetrain.changeRotationAssistance(), m_drivetrain)
    )
    .onFalse(
      new InstantCommand(() -> m_drivetrain.changeRotationAssistance(), m_drivetrain)
    );

    // Left Bump
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_leftbump)
    .whileTrue(
      new AssistedShootCommand(m_servoSubsystem1, m_servoSubsystem2, m_shootSubsystem)
    );

    */

    // End of Testing

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

    /*
    // X - Hub Shot
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_X)
    .onTrue(
      new RunCommand(() -> m_shootSubsystem.shoot(), m_shootSubsystem)
    )
    .onFalse(
      new InstantCommand(() -> m_shootSubsystem.stopShooter(), m_shootSubsystem)
    );
    */

    // Left Trig - Shoot Rev Up
    new Trigger(() -> m_operatorController.getRawAxis(ControllerConstants.k_lefttrig) > 0.05)
    .whileTrue(
        new ParallelDeadlineGroup(
          new AssistedShootCommand(m_servoSubsystem1, m_servoSubsystem2, m_shootSubsystem), 
          new LEDColorChangeCommand(m_LEDSubsystem, m_shootSubsystem, m_servoSubsystem1, m_servoSubsystem2)
        )
    );

    // Right Trig - Auto Align
    new Trigger(() -> m_operatorController.getRawAxis(ControllerConstants.k_righttrig) > 0.05)
    .whileTrue(
      new ChangeRotationAssistanceCommand()
    );

    // Y Operator - Reverse Intake
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_Y)
    .onTrue(
      new RunCommand(() -> m_intakeSubsystem.reverseIntake(), m_intakeSubsystem)
    )
    .onFalse(
      new InstantCommand(() -> m_intakeSubsystem.stop(), m_intakeSubsystem)
    );

    // A - Intakedex

    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_A)
    .whileTrue(
        new SequentialCommandGroup(
        new SetIntakeFlipoutCommand(m_flipoutSubsystem, "INTAKEDEX UPPER"),
        new WaitCommand(0.3),
        new SetIntakeFlipoutCommand(m_flipoutSubsystem, "INTAKEDEX LOWER"),
        new WaitCommand(0.3)).repeatedly()
    );

    // B - Zero Flipout

    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_B)
    .onTrue(
      new ForceZeroFlipoutCommand(m_flipoutSubsystem, IntakeConstants.k_intakeRetractedAngle)
    );

    // X - Set Flipout to "Deploy"
    
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.k_X)
    .onTrue(
      new ForceZeroFlipoutCommand(m_flipoutSubsystem, IntakeConstants.k_intakeDeployedAngle)
    );
    

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
      m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public SequentialCommandGroup DeployflipoutCommandGroup() {
    return new SequentialCommandGroup(
      new SetIntakeFlipoutCommand(m_flipoutSubsystem, "DEPLOY"),
      new ZeroFlipoutCommand(m_flipoutSubsystem, IntakeConstants.k_intakeDeployedAngle)
    );
  }  

  public SequentialCommandGroup AutoIntakedexCommandGroup() {
    return new SequentialCommandGroup(
      new SetIntakeFlipoutCommand(m_flipoutSubsystem, "STOW")
    );
  } 
}
