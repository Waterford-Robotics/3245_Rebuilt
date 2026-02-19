package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

// Move Robot to Target Pose
public class PathfindToPoseCommand extends Command {
    
  // Instantiate Stuff
  CommandSwerveDrivetrain m_driveSubsystem;

  // Target Pose
  Pose2d m_targetPose;

  // Constraints
  PathConstraints m_constraints;

  // Command
  Command m_pathfindingCommand;

  // Constructor
  public PathfindToPoseCommand(CommandSwerveDrivetrain driveSubsystem, Pose2d targetPose, PathConstraints constraints) {
        
    // Definitions and setting parameters are equal to members!
    m_driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);

    m_targetPose = targetPose;
    m_constraints = constraints;
  }

  // What we do to set up the command
  public void initialize() {

    // Build the path automatically using Pathplanner
    m_pathfindingCommand = AutoBuilder.pathfindToPose(
      m_targetPose, 
      m_constraints, 
      0.0
    );
  }
    
  // The actual control!
  public void execute() {

    // RUN IT!
    m_pathfindingCommand.schedule();
  }

  // Add stuff we do after to reset here 
  public void end(boolean interrupted) {}

  // Are we done yet? Finishes immediately
  public boolean isFinished() {
    return true;
  }
}
