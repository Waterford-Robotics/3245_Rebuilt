
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeFlipoutSubsystem;

// Raises Wrist
public class ZeroFlipoutCommand extends Command {

  // Uses Wrist and Subsystems
  IntakeFlipoutSubsystem m_flipout;
  boolean m_finished;

  // Constructor
  public ZeroFlipoutCommand(IntakeFlipoutSubsystem flipout) {
        
    // Definitions and setting parameters are equal to members!
    m_flipout = flipout;
    addRequirements(flipout);
  }

  // Reset timer when the command starts executing
  public void initialize() {
    m_finished = false;
  }
  
  // Actual command
  public void execute() {
    if(m_flipout.getCurrentPosition() < 0.15 && m_flipout.getCurrentVelocity() == 0) {
      m_finished = true;
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {

    // Neutral Motors and Reset Encoder Values
    m_flipout.setNeutral();
    m_flipout.resetSensorPosition(IntakeConstants.k_intakeRetractedAngle);
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return m_finished;
  }
}
