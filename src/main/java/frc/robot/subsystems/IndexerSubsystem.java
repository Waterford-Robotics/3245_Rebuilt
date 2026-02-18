// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  private TalonFX m_indexer;

  private TalonFXConfiguration indexerConfig;

  double indexSpeed;
  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
     m_indexer = new TalonFX(21, "Mechanisms");

    indexerConfig = new TalonFXConfiguration();

    // Kraken Configs
    indexerConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.05;
    indexerConfig.MotorOutput.PeakForwardDutyCycle = 0.4;
    indexerConfig.MotorOutput.PeakReverseDutyCycle = -0.4;
    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexerConfig.CurrentLimits.SupplyCurrentLimit = 40;
    SmartDashboard.putNumber("Index Speed Percent", 25);
  }

  public void index(){
    indexSpeed = SmartDashboard.getNumber("Index Speed Percent", 25);
    m_indexer.set(indexSpeed/100);
  }

  public void index(double indexSpeed){
    m_indexer.set(indexSpeed/100);
  }

  public void stopIndexer(){
    m_indexer.set(0);
  }

}
