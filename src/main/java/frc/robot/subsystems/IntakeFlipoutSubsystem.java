package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.MotorIDConstants;

public class IntakeFlipoutSubsystem extends SubsystemBase {
     
  private TalonFX m_intakeFlipout1; // Left?
	private TalonFX m_intakeFlipout2; // Right?

  public IntakeFlipoutSubsystem() {

	  // Config stuff yeah
		m_intakeFlipout1 = new TalonFX(MotorIDConstants.k_intakeFlipoutLeftID, "Mechanisms"); 
		m_intakeFlipout2 = new TalonFX(MotorIDConstants.k_intakeFlipoutRightID, "Mechanisms"); 

		m_intakeFlipout1.getConfigurator().apply(IntakeConfigs.INTAKE_FLIPOUT_TALON_FX_CONFIGURATION, 0.05);
		m_intakeFlipout2.getConfigurator().apply(IntakeConfigs.INTAKE_FLIPOUT_TALON_FX_CONFIGURATION, 0.05);
	}

  // Set the Position
  public void setPosition(Angle angle) {

    // Leader
    m_intakeFlipout1.setControl(new PositionVoltage(angle.in(Units.Rotations)));

    // Follower
    m_intakeFlipout2.setControl(new Follower(m_intakeFlipout1.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  // Neutralize motors
  public void setNeutral() {
    m_intakeFlipout1.setControl(new NeutralOut());
    m_intakeFlipout2.setControl(new NeutralOut());
  }

  // Reset for zeroing
  public void resetSensorPosition(Angle setpoint) {
    m_intakeFlipout1.setPosition(setpoint.in(Units.Rotations));
    m_intakeFlipout2.setPosition(setpoint.in(Units.Rotations));
  }

  // Get the Current Flipout Pos
  public double getCurrentPosition() {
    return m_intakeFlipout1.getPosition().getValueAsDouble();
  }

  // Get the Current Flipout Velo
  public double getCurrentVelocity() {
    return m_intakeFlipout1.getVelocity().getValueAsDouble();
  }

  // Smartdash stuff
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flipout/Pos", Units.Rotations.of(m_intakeFlipout1.getPosition().getValueAsDouble()).magnitude());
    /*
    SmartDashboard.putString("Flipout/Units", m_intakeFlipout.getPosition().getUnits());
    SmartDashboard.putNumber("Flipout/CLO", m_intakeFlipout.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Flipout/Output", m_intakeFlipout.get());
    SmartDashboard.putNumber("Flipout/Inverted", m_intakeFlipout.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Flipout/Current", m_intakeFlipout.getSupplyCurrent().getValueAsDouble());
    */
  }
}
