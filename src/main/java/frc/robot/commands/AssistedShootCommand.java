// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AssistedShootCommand extends Command {

  CommandSwerveDrivetrain m_drivetrain;
  ServoSubsystem m_servoSubsystem1;
  ServoSubsystem m_servoSubsystem2;
  ShootSubsystem m_shootSubsystem;
  /** Creates a new AssistedShootCommand. */
  public AssistedShootCommand(CommandSwerveDrivetrain drivetrain, ServoSubsystem servoSubsystem1,
              ServoSubsystem servoSubsystem2, ShootSubsystem shootSubsystem) {
    m_drivetrain = drivetrain;
    m_servoSubsystem1 = servoSubsystem1;
    m_servoSubsystem2 = servoSubsystem2;
    m_shootSubsystem = shootSubsystem;

    addRequirements(drivetrain);
    addRequirements(servoSubsystem1);
    addRequirements(servoSubsystem2);
    addRequirements(shootSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_servoSubsystem1.setPosition(m_drivetrain.getServoAngle(m_drivetrain.getDistanceToHubCenter()));
    m_servoSubsystem2.setPosition(m_drivetrain.getServoAngle(m_drivetrain.getDistanceToHubCenter()));
    m_shootSubsystem.shoot(m_drivetrain.getShooterSpeed(m_drivetrain.getDistanceToHubCenter()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shootSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
