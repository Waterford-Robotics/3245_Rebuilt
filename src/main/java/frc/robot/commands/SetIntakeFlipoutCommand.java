package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeFlipoutSubsystem;

public class SetIntakeFlipoutCommand extends Command {

    IntakeFlipoutSubsystem m_flipout;
    String m_position;

    public SetIntakeFlipoutCommand(IntakeFlipoutSubsystem flipout, String position) {
        m_flipout = flipout;

        m_position = position;
        addRequirements(flipout);
    }

    public void initialize() {}
    
    public void execute() {
        if (m_position.equals("STOW")){
            m_flipout.setPosition(IntakeConstants.k_intakeRetractedAngle);
        }

        if (m_position.equals("INTAKEDEX UPPER")){
            m_flipout.setPosition(IntakeConstants.k_intakedexRetractedAngle);
        }

        if (m_position.equals("INTAKEDEX LOWER")){
            m_flipout.setPosition(IntakeConstants.k_intakedexDeployedAngle);
        }

        if (m_position.equals("DEPLOY")){
            m_flipout.setPosition(IntakeConstants.k_intakeDeployedAngle);
        }
        
    }

    // Stuff that happens when command is over
  public void end(boolean interrupted) {
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return true;
  }
}
