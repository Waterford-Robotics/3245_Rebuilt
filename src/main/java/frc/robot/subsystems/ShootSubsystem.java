package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShootSubsystem extends SubsystemBase{
  /*
   2 meters = (10,25)
   2.5 meters = (16, 25.75)
   3 meters = (20, 26.5)
   3.5 meters = (24, 27.25)
   4 meters = (28, 28)
   4.5 meters = (32, 28.75)
   */

  private TalonFX m_shooter1;
  private TalonFX m_shooter2;

  private TalonFXConfiguration shooterConfig;
  double shootSpeed;

  public ShootSubsystem() {
    
    m_shooter1 = new TalonFX(22, "Mechanisms");//left
    m_shooter2 = new TalonFX(23, "Mechanisms");//right

    shooterConfig = new TalonFXConfiguration();

    // Kraken Configs
    shooterConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.05;
    shooterConfig.MotorOutput.PeakForwardDutyCycle = 0.4;
    shooterConfig.MotorOutput.PeakReverseDutyCycle = -0.4;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = 40;

    m_shooter1.getConfigurator().apply(shooterConfig, 0.05);
    m_shooter2.getConfigurator().apply(shooterConfig, 0.05);

  }

  public void shoot(){
    shootSpeed = SmartDashboard.getNumber("Shoot Speed Percent", 30);
    m_shooter1.set(-shootSpeed/100);
    m_shooter2.set(shootSpeed/100);
  }

  public void shoot(double shootSpeed){
    m_shooter1.set(-shootSpeed/100);
    m_shooter2.set(shootSpeed/100);
  }


  public void stopShooter() {
    m_shooter1.set(0);
    m_shooter2.set(0);
  }

  public void periodic() {
  }
}
