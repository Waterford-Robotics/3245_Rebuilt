package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private TalonFX m_intake;
    private TalonFX m_flipout;
    double intakeSpeed;

    public IntakeSubsystem(){
        m_intake = new TalonFX(IntakeConstants.k_intakeID, "Mechanisms"); 
        m_flipout = new TalonFX(IntakeConstants.k_flipoutID, "Mechanisms");

        // Kraken Configs
        m_intake.getConfigurator().apply(IntakeConfigs.INTAKE_TALON_FX_CONFIGURATION, 0.05);
        m_flipout.getConfigurator().apply(IntakeConfigs.INTAKE_TALON_FX_CONFIGURATION, 0.05);
        SmartDashboard.putNumber("Intake Speed Percent", 25);
      
    }
    
    public void intake(){
        intakeSpeed = SmartDashboard.getNumber("Intake Speed Percent", 25);
        m_intake.set(-intakeSpeed/100);
    }

    public void stop() {
        m_intake.set(0);
    }

    public void periodic() {}  
    
}
