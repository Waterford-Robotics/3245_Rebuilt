package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorIDConstants;

public class IntakeSubsystem extends SubsystemBase {

	// Kraken x44
	private TalonFX m_intakeRoller;

	public IntakeSubsystem() {
	  // Config stuff yeah
		m_intakeRoller = new TalonFX(MotorIDConstants.k_intakeRollerID, "Mechanisms"); 

		m_intakeRoller.getConfigurator().apply(IntakeConfigs.INTAKE_ROLLER_TALON_FX_CONFIGURATION, 0.05);
	}
	
	// Vroom vroom
	public void intake() {
		m_intakeRoller.set(IntakeConstants.k_intakeSpeed);
	}

	public void reverseIntake() {
		m_intakeRoller.set(-IntakeConstants.k_intakeSpeed);
	}

	// STOP!
	public void stop() {
		m_intakeRoller.set(0);
	}

  // Lowk can be SmartDash telemetry stuff
	public void periodic() {}  
}
