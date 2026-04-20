package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

// Move Robot to Target Pose
public class AutoRotationAssistanceCommand extends Command {

  CommandSwerveDrivetrain m_drivetrain;
  // Constructor
  public AutoRotationAssistanceCommand(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // What we do to set up the command
  public void initialize() {
  }
    
  // The actual control!
  public void execute() {
    PPHolonomicDriveController.overrideRotationFeedback(() -> {return m_drivetrain.getAdjustedRotation(0)*DriveConstants.k_maxAngularRate;});
  }

  // Add stuff we do after to reset here 
  public void end(boolean interrupted) {
    m_drivetrain.changeRotationAssistance(false);
    PPHolonomicDriveController.clearRotationFeedbackOverride();
  }

  // Are we done yet? Finishes immediately
  public boolean isFinished() {
    return false;
  }
}
