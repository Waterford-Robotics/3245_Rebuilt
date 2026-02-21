// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;

public class RollerSubsystem extends SubsystemBase {
  private TalonFX m_roller;

  double rollerSpeed;
  /** Creates a new rollererSubsystem. */
  public RollerSubsystem() {
     m_roller = new TalonFX(28, "Mechanisms");

    m_roller.getConfigurator().apply(IntakeConfigs.INTAKE_TALON_FX_CONFIGURATION, 0.05);
    SmartDashboard.putNumber("roller Speed Percent", 25);
  }

  public void roller() {
    rollerSpeed = SmartDashboard.getNumber("roller Speed Percent", 25);
    m_roller.set(rollerSpeed/100);
  }

  public void roller(double rollerSpeed){
    m_roller.set(rollerSpeed/100);
  }

  public void stopRoller(){
    m_roller.set(0);
  }

}
