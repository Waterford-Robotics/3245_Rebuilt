// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LiveConstants;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LEDColorChangeCommand extends Command {

//   private boolean m_status;
  LEDSubsystem LEDs;
  Timer m_timer = new Timer();

  /** Creates a new AssistedShootCommand. */
  public LEDColorChangeCommand(boolean statusBoolean) {
    
    // m_status = statusBoolean;
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    LEDs.setRed();
    m_timer.start();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    // LiveConstants._enableShooter = m_status;
    if(m_timer.get() > LEDConstants.k_timeBeforeRevComplete) {
      LEDs.setGreen();
    } else {
      LEDs.setRed();
    }
  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    // LEDs.setStrobeGold();
  }

  // Returns true when the command should end.
  public boolean isFinished() {
    return true;
  }
}
