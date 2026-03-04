package frc.robot;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShootConstants;

public class Configs {

  /*
   ********************************************
   **    SHOOT KRAKEN x60 CONFIGURATIONS     **
   ********************************************
  */
  public static final class ShootConfigs {

    // Shooter Kraken x60
    public static final TalonFXConfiguration SHOOT_TALON_FX_CONFIGURATION = new TalonFXConfiguration();

    // Indexer Kraken x60
    public static final TalonFXConfiguration INDEXER_TALON_FX_CONFIGURATION = new TalonFXConfiguration();

    // Conveyor CANRange
    public static final CANrangeConfiguration SHOOT_CANRANGE_CONFIGURATION = new CANrangeConfiguration();
    
    static {
      SHOOT_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = ShootConstants.k_shooterRampRate;
      SHOOT_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = ShootConstants.k_shooterClosedMaxSpeed;
      SHOOT_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -ShootConstants.k_shooterClosedMaxSpeed;
      SHOOT_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      SHOOT_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = ShootConstants.k_shooterSupplyCurrentLimit;

      INDEXER_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = ShootConstants.k_shooterRampRate;
      INDEXER_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = ShootConstants.k_shooterClosedMaxSpeed;
      INDEXER_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -ShootConstants.k_shooterClosedMaxSpeed;
      INDEXER_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      INDEXER_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = ShootConstants.k_shooterSupplyCurrentLimit;

      SHOOT_CANRANGE_CONFIGURATION.ProximityParams.MinSignalStrengthForValidMeasurement = 2500; // TODO: Make it bigger?
      SHOOT_CANRANGE_CONFIGURATION.ProximityParams.ProximityThreshold = 0.1; // TODO: Tune for status Lights
      SHOOT_CANRANGE_CONFIGURATION.ProximityParams.ProximityHysteresis = 0.01; 
      SHOOT_CANRANGE_CONFIGURATION.FovParams.FOVRangeX = 6.75;
      SHOOT_CANRANGE_CONFIGURATION.FovParams.FOVRangeY = 6.75; 
      SHOOT_CANRANGE_CONFIGURATION.ToFParams.UpdateFrequency = 50;
      SHOOT_CANRANGE_CONFIGURATION.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;   
    }
  } 

  /*
  ********************************************
  **    INTAKE KRAKEN x44 CONFIGURATIONS    **
  ********************************************
  */
  public static final class IntakeConfigs {

    // Intake Kraken x44
    public static final TalonFXConfiguration INTAKE_TALON_FX_CONFIGURATION = new TalonFXConfiguration();

    static{
      INTAKE_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = IntakeConstants.k_intakeRampRate;
      INTAKE_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = IntakeConstants.k_intakeClosedMaxSpeed;
      INTAKE_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -IntakeConstants.k_intakeClosedMaxSpeed;
      INTAKE_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      INTAKE_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = IntakeConstants.k_intakeSupplyCurrentLimit;
    }
  }

  /*
   ********************************************
   **    INDEXER KRAKEN x44 CONFIGURATIONS   **
   ********************************************
   */
  public static final class IndexConfigs {

    // Shooter Kraken x44
    public static final TalonFXConfiguration ROLLER_INDEXER_TALON_FX_CONFIGURATION = new TalonFXConfiguration();
    public static final TalonFXConfiguration SHOOTER_INDEXER_TALON_FX_CONFIGURATION = new TalonFXConfiguration();

    static {
      
      ROLLER_INDEXER_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = IndexerConstants.k_rollerIndexerRampRate;
      ROLLER_INDEXER_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = IndexerConstants.k_rollerIndexerClosedMaxSpeed;
      ROLLER_INDEXER_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -IndexerConstants.k_rollerIndexerClosedMaxSpeed;
      ROLLER_INDEXER_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ROLLER_INDEXER_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = IndexerConstants.k_rollerIndexerSupplyCurrentLimit;

      SHOOTER_INDEXER_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = IndexerConstants.k_rollerIndexerRampRate;
      SHOOTER_INDEXER_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = IndexerConstants.k_rollerIndexerClosedMaxSpeed;
      SHOOTER_INDEXER_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -IndexerConstants.k_rollerIndexerClosedMaxSpeed;
      SHOOTER_INDEXER_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      SHOOTER_INDEXER_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = IndexerConstants.k_rollerIndexerSupplyCurrentLimit;
    }
  }
}