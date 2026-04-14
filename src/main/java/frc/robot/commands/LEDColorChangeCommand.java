// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.*;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LEDColorChangeCommand extends Command {

//   private boolean m_status;
  LEDSubsystem m_ledSubsystem;

  ShootSubsystem m_shootSubsystem;
  ServoSubsystem m_servoSubsystem1;
  ServoSubsystem m_servoSubsystem2;



  /** Creates a new LEDColorChangeCommand. */
  public LEDColorChangeCommand(LEDSubsystem LEDs, ShootSubsystem shootSubsystem, ServoSubsystem servoSubsystem1, ServoSubsystem servoSubsystem2) {
    m_ledSubsystem = LEDs;
    m_shootSubsystem = shootSubsystem;
    m_servoSubsystem1 = servoSubsystem1;
    m_servoSubsystem2 = servoSubsystem2;

    addRequirements(LEDs);
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    m_ledSubsystem.setRainbowParty();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    SmartDashboard.putNumber("Shooter Reading Speed", m_shootSubsystem.getSpeed());
    // LiveConstants._enableShooter = m_status;
    if((Math.abs(RobotContainer.m_drivetrain.getShooterSpeed(RobotContainer.m_drivetrain.getDistanceToHubCenter()) - m_shootSubsystem.getSpeed()) < 0.05)){
        //&& (Math.abs(RobotContainer.m_drivetrain.getServoAngle(RobotContainer.m_drivetrain.getDistanceToHubCenter()) - m_servoSubsystem1.getPosition())<5) 
        //&& (Math.abs(RobotContainer.m_drivetrain.getServoAngle(RobotContainer.m_drivetrain.getDistanceToHubCenter()) - m_servoSubsystem2.getPosition())<5)){
      m_ledSubsystem.setGreen();
    }
    else{
      m_ledSubsystem.setRed();
    }

  }
  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    m_ledSubsystem.setRainbowParty();
  }

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }
}