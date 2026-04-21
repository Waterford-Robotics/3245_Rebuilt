// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ServoSubsystem;
import frc.robot.subsystems.ShootSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualShootForSecsCommand extends Command {

  ServoSubsystem m_servoSubsystem1;
  ServoSubsystem m_servoSubsystem2;
  ShootSubsystem m_shootSubsystem;

  Timer m_timer;
  double m_seconds;
  
  double m_shooterSpeed;
  double m_servoPos;
  
  /** Creates a new AssistedShootCommand. */
  public ManualShootForSecsCommand(ServoSubsystem servoSubsystem1, ServoSubsystem servoSubsystem2, ShootSubsystem shootSubsystem, double shooterSpeed, double servoPos, double seconds) {

    m_servoSubsystem1 = servoSubsystem1;
    m_servoSubsystem2 = servoSubsystem2;
    m_shootSubsystem = shootSubsystem;

    addRequirements(servoSubsystem1);
    addRequirements(servoSubsystem2);
    addRequirements(shootSubsystem);

    m_shooterSpeed = shooterSpeed;
    m_servoPos = servoPos;

    m_timer = new Timer();
    m_seconds = seconds;
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    m_timer.start();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled
  public void execute() {
    m_servoSubsystem1.setPosition(m_servoPos);
    m_servoSubsystem2.setPosition(m_servoPos);
    m_shootSubsystem.shoot(m_shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shootSubsystem.stopShooter();
    m_servoSubsystem1.setPosition(0);
    m_servoSubsystem2.setPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_seconds);
  }
}
