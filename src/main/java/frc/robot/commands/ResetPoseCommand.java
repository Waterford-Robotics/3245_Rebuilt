package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

// Move Robot to Target Pose
public class ResetPoseCommand extends Command {
    
  // Instantiate Stuff
  CommandSwerveDrivetrain m_driveSubsystem;

  Pose2d m_pose;

  // Constructor
  public ResetPoseCommand(CommandSwerveDrivetrain driveSubsystem, Pose2d pose) {
        
    // Definitions and setting parameters are equal to members!
    m_driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);

    m_pose = pose;

  }

  // What we do to set up the command
  public void initialize() {

    m_driveSubsystem.resetPose(m_pose);
  }
    
  // The actual control!
  public void execute() {
  }

  // Add stuff we do after to reset here 
  public void end(boolean interrupted) {
  }

  // Are we done yet? Finishes immediately
  public boolean isFinished() {
    return true;
  }
}
