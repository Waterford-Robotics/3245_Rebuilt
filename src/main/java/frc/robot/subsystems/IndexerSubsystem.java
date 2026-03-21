// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IndexConfigs;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.MotorIDConstants;

public class IndexerSubsystem extends SubsystemBase {

  private TalonFX m_transferIndexer;
  private TalonFX m_rollerFloor;

  public IndexerSubsystem() {

    m_transferIndexer = new TalonFX(MotorIDConstants.k_transferIndexerID, "Mechanisms");
    m_rollerFloor = new TalonFX(MotorIDConstants.k_rollerFloorID, "Mechanisms");

    m_transferIndexer.getConfigurator().apply(IndexConfigs.TRANSFER_INDEXER_TALON_FX_CONFIGURATION, 0.05);
    m_rollerFloor.getConfigurator().apply(IndexConfigs.ROLLER_FLOOR_TALON_FX_CONFIGURATION, 0.05);
  }

  // Index Fuels
  public void index() {
    m_transferIndexer.set(IndexerConstants.k_indexerSpeed);
    m_rollerFloor.set(IndexerConstants.k_indexerSpeed);
  }

  // Run indexers backwards
  public void reverseIndex() {
    m_transferIndexer.set(-IndexerConstants.k_indexerSpeed);
    m_rollerFloor.set(-IndexerConstants.k_indexerSpeed);
  }

  // Index at custom speed
  public void index(double indexSpeed) {
    m_transferIndexer.set(indexSpeed);
    m_rollerFloor.set(indexSpeed);
  }

  // STOP! Hammer time
  public void stopIndexer() {
    m_transferIndexer.set(0);
    m_rollerFloor.set(0);
  }

  public void periodic() {}
}
