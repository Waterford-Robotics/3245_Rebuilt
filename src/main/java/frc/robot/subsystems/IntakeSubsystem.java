package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorIDConstants;

public class IntakeSubsystem extends SubsystemBase {

	// Kraken x44 [x2]
	private TalonFX m_intakeRoller;
	private TalonFX m_innerIntake;

	public IntakeSubsystem() {
	  // Config stuff yeah
		m_intakeRoller = new TalonFX(MotorIDConstants.k_intakeRollerID, "Mechanisms"); 
		m_innerIntake = new TalonFX(MotorIDConstants.k_innerIntakeID, "Mechanisms"); 

		m_intakeRoller.getConfigurator().apply(IntakeConfigs.INTAKE_TALON_FX_CONFIGURATION, 0.05);
		m_intakeRoller.getConfigurator().apply(IntakeConfigs.INTAKE_TALON_FX_CONFIGURATION, 0.05);
	}
	
	// Vroom vroom
	public void intake() {
		m_intakeRoller.set(IntakeConstants.k_intakeSpeed);
		m_innerIntake.set(-IntakeConstants.k_intakeSpeed);
	}

	// STOP!
	public void stop() {
		m_intakeRoller.set(0);
		m_innerIntake.set(0);
	}

  // Lowk can be SmartDash telemetry stuff
	public void periodic() {}  
}
