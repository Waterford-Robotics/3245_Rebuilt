package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ShootConfigs;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.ShootConstants;

public class ShootSubsystem extends SubsystemBase{
  /*
   2 meters = (10,25)
   2.5 meters = (16, 25.75)
   3 meters = (20, 26.5)
   3.5 meters = (24, 27.25)
   4 meters = (28, 28)
   4.5 meters = (32, 28.75)
   */

  private TalonFX m_shooterLeft;
  private TalonFX m_shooterRight;

  double shootSpeed;

  public ShootSubsystem() {
    
    m_shooterLeft = new TalonFX(MotorIDConstants.k_shooterLeftID, "Mechanisms");
    m_shooterRight = new TalonFX(MotorIDConstants.k_shooterRightID, "Mechanisms");

    m_shooterLeft.getConfigurator().apply(ShootConfigs.SHOOT_TALON_FX_CONFIGURATION, 0.05);
    m_shooterRight.getConfigurator().apply(ShootConfigs.SHOOT_TALON_FX_CONFIGURATION, 0.05);
  }

  public void shoot(){
    m_shooterLeft.set(-ShootConstants.k_shooterSpeed);
    m_shooterRight.set(ShootConstants.k_shooterSpeed);
  }

  public void shoot(double shootSpeed){
    m_shooterLeft.set(-shootSpeed);
    m_shooterRight.set(shootSpeed);
  }


  public void stopShooter() {
    m_shooterLeft.set(0);
    m_shooterRight.set(0);
  }

  public void periodic() {}
}
