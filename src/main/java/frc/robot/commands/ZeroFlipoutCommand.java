
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeFlipoutSubsystem;

// Raises Wrist
public class ZeroFlipoutCommand extends Command {

  // Uses Wrist and Subsystems
  IntakeFlipoutSubsystem m_flipout;
  boolean m_finished;
  Angle m_angle;

  // Constructor
  public ZeroFlipoutCommand(IntakeFlipoutSubsystem flipout, Angle angle) {
        
    // Definitions and setting parameters are equal to members!
    m_flipout = flipout;
    addRequirements(flipout);
    m_angle = angle;
  }

  // Reset timer when the command starts executing
  public void initialize() {
    m_finished = false;
  }
  
  // Actual command
  public void execute() {
    SmartDashboard.putNumber("Flipout Error", Math.abs(m_flipout.getCurrentPosition() - m_angle.in(Rotations)));
    if(Math.abs(m_flipout.getCurrentPosition() - m_angle.in(Rotations)) < 0.75 && m_flipout.getCurrentVelocity() == 0) {
      m_finished = true;
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {

    // Neutral Motors and Reset Encoder Values
    m_flipout.setNeutral();
    m_flipout.resetSensorPosition(m_angle);
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return m_finished;
  }
}
