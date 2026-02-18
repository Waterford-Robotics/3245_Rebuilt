package frc.robot;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import frc.robot.Constants.ShootConstants;

public class Configs {
  public static final class ShootConfigs {
  // Shooter Kraken x60
  public static final TalonFXConfiguration SHOOT_TALON_FX_CONFIGURATION = new TalonFXConfiguration();

  // Conveyor CANRange
  public static final CANrangeConfiguration SHOOT_CANRANGE_CONFIGURATION = new CANrangeConfiguration();
  static{
      /*
      ********************************************
      **    SHOOT KRAKEN x60 CONFIGURATIONS    **
      ********************************************
    */

    SHOOT_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = ShootConstants.k_shooterRampRate;
    SHOOT_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = ShootConstants.k_shooterClosedMaxSpeed;
    SHOOT_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -ShootConstants.k_shooterClosedMaxSpeed;
    SHOOT_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    SHOOT_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = ShootConstants.k_shooterSupplyCurrentLimit;

      /*
      ******************************************
      **    SHOOT CANRANGE CONFIGURATIONS    **
      ******************************************
    */

    SHOOT_CANRANGE_CONFIGURATION.ProximityParams.MinSignalStrengthForValidMeasurement = 2500; // TODO: Make it bigger?
    SHOOT_CANRANGE_CONFIGURATION.ProximityParams.ProximityThreshold = 0.1; // TODO: Tune for status Lights
    SHOOT_CANRANGE_CONFIGURATION.ProximityParams.ProximityHysteresis = 0.01;
    
    SHOOT_CANRANGE_CONFIGURATION.ToFParams.UpdateFrequency = 50;
    SHOOT_CANRANGE_CONFIGURATION.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;   
  }
} 
}