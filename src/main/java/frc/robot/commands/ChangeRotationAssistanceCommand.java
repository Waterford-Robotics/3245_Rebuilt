package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

// Move Robot to Target Pose
public class ChangeRotationAssistanceCommand extends Command {
    

  // Constructor
  public ChangeRotationAssistanceCommand() {

  }

  // What we do to set up the command
  public void initialize() {

    RobotContainer.m_drivetrain.changeRotationAssistance(true);
  }
    
  // The actual control!
  public void execute() {
  }

  // Add stuff we do after to reset here 
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrain.changeRotationAssistance(false);
  }

  // Are we done yet? Finishes immediately
  public boolean isFinished() {
    return false;
  }
}
