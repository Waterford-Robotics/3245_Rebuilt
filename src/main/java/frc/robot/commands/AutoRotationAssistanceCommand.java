//DOES NOT WORK




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
  public AutoRotationAssistanceCommand() {
    m_drivetrain = RobotContainer.m_drivetrain;
  }

  // What we do to set up the command
  public void initialize() {
    m_drivetrain.changeRotationAssistance(true);
    PPHolonomicDriveController.overrideRotationFeedback(() -> {return m_drivetrain.getAdjustedRotation(0)*DriveConstants.k_maxAngularRate;});
  }
    
  // The actual control!
  public void execute() {
    /* 
    m_drivetrain.applyRequest(() -> 
        RobotContainer.drive.withRotationalRate(m_drivetrain.getAdjustedRotation(0)* DriveConstants.k_maxAngularRate)
    );
    */

    
    //PPHolonomicDriveController.overrideRotationFeedback(() -> {return 2.0;});
    /*
     m_drivetrain.applyRequest(() -> 
        RobotContainer.drive.withRotationalRate(2.0)
      );
     */

  }

  // Add stuff we do after to reset here 
  public void end(boolean interrupted) {
    m_drivetrain.changeRotationAssistance(false);
    /* 
    m_drivetrain.applyRequest(() -> 
        RobotContainer.drive.withRotationalRate(0)
    );
    */
    PPHolonomicDriveController.clearRotationFeedbackOverride();
  }

  // Are we done yet? Finishes immediately
  public boolean isFinished() {
    return false;
  }
}
