// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Configs.ShootConfigs;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  private TalonFX m_shooterIndexer;
  private TalonFX m_rollerIndexer;

  double indexSpeed;
  /** Creates a new rollererSubsystem. */
  public IndexerSubsystem() {
     m_shooterIndexer = new TalonFX(IndexerConstants.k_shooterIndexerID, "Mechanisms");
     m_rollerIndexer = new TalonFX(IndexerConstants.k_rollerIndexerID, "Mechanisms");

    m_shooterIndexer.getConfigurator().apply(ShootConfigs.INDEXER_TALON_FX_CONFIGURATION, 0.05);
    m_rollerIndexer.getConfigurator().apply(IntakeConfigs.INTAKE_TALON_FX_CONFIGURATION, 0.05);


    SmartDashboard.putNumber("Index Speed Percent", 25);
  }

  public void index() {
    indexSpeed = SmartDashboard.getNumber("Index Speed Percent", 25);
    m_shooterIndexer.set(indexSpeed/100);
    m_rollerIndexer.set(indexSpeed/100);
  }

  public void index(double indexSpeed){
    m_shooterIndexer.set(indexSpeed/100);
    m_rollerIndexer.set(indexSpeed/100);
  }

  public void stopIndexer(){
    m_shooterIndexer.set(0);
    m_rollerIndexer.set(0);
  }

}
