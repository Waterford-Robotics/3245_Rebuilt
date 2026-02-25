// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ShootConfigs;

public class IndexerSubsystem extends SubsystemBase {
  private TalonFX m_indexer;


  private CANRangeSubsystem m_CanRangeSubsystem;

  double indexSpeed;
  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem(CANRangeSubsystem canRangeSubsystem) {
     m_indexer = new TalonFX(23, "Mechanisms");
     m_CanRangeSubsystem = canRangeSubsystem;

    m_indexer.getConfigurator().apply(ShootConfigs.INDEXER_TALON_FX_CONFIGURATION, 0.05);
    SmartDashboard.putNumber("Index Speed Percent", 25);
  }

  public void index() {
    indexSpeed = SmartDashboard.getNumber("Index Speed Percent", 25);
    m_indexer.set(indexSpeed/100);
  }
  public void indexWithBreak(){
    indexSpeed = SmartDashboard.getNumber("Index Speed Percent", 25);
    if (!m_CanRangeSubsystem.getIsDetected()){
      m_indexer.set(indexSpeed/100);
    }
    else {
      stopIndexer();
    }
  }

  public void index(double indexSpeed){
    m_indexer.set(indexSpeed/100);
  }

  public void stopIndexer(){
    m_indexer.set(0);
  }

}
