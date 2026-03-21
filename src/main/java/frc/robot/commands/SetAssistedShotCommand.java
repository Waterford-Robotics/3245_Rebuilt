// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LiveConstants;

public class SetAssistedShotCommand extends Command {

  private boolean m_status;

  /** Creates a new AssistedShootCommand. */
  public SetAssistedShotCommand(boolean statusBoolean) {
    
    m_status = statusBoolean;
  }

  // Called when the command is initially scheduled.
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    LiveConstants._enableShooter = m_status;
  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  public boolean isFinished() {
    return true;
  }
}
