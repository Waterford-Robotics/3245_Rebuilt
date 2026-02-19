package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

// Move Robot to Target Pose
public class PathfindThenFollowPathCommand extends Command {
    
  // Instantiate Stuff
  CommandSwerveDrivetrain m_driveSubsystem;

  // Target Path Name
  String m_pathName;

  // Target Path
  PathPlannerPath m_path;

  // Constraints
  PathConstraints m_constraints;

  // Command
  Command m_pathfindingCommand;

  // Constructor
  public PathfindThenFollowPathCommand(CommandSwerveDrivetrain driveSubsystem, String pathName, PathConstraints constraints) {
        
    // Definitions and setting parameters are equal to members!
    m_driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);

    m_pathName = pathName;
    m_constraints = constraints;
  }

  // What we do to set up the command
  public void initialize() {

    try {
      m_path = PathPlannerPath.fromPathFile(m_pathName);
      m_pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
        m_path, 
        m_constraints
      );
    }
    catch(Exception e) {
      e.printStackTrace();
    }
  }
    
  // The actual control!
  public void execute() {

    // RUN IT!
    try {
      m_pathfindingCommand.schedule();
    }
    catch(Exception e) {
      e.printStackTrace();
    }
  }

  // Add stuff we do after to reset here 
  public void end(boolean interrupted) {}

  // Are we done yet? Finishes immediately
  public boolean isFinished() {
    return true;
  }
}
