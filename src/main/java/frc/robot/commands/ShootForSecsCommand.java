// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShootSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

// Runs Shooter for a Certain Number of Seconds
public class ShootForSecsCommand extends Command {

  // Uses Shooter
  ShootSubsystem m_shootSubsystem;
  LEDSubsystem LEDs;
  double m_seconds;
  Timer m_timer = new Timer();


  // Constructor
  public ShootForSecsCommand(ShootSubsystem shootSubsystem, double seconds) {
        
    // Definitions and setting parameters are equal to members!
    m_shootSubsystem = shootSubsystem;
    addRequirements(shootSubsystem);

    // Time
    m_seconds = seconds;
  }

  // Reset timer when the command starts executing
  public void initialize() {
    LEDs.setRed();
    m_timer.start();
    m_timer.reset();
  }
  
  // Actual command
  public void execute() {


    if(m_timer.get() > k_timeBeforeRevComplete.get() && m_timer.get() < m_seconds) {
      LEDs.setGreen();
    } else {
      LEDs.setRed();
    }
    if(m_timer.get() < m_seconds) {
      m_shootSubsystem.shoot();
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {
    m_shootSubsystem.stopShooter();
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return m_timer.get() > m_seconds;
    // LEDs.setStrobeGold();
  }
}
