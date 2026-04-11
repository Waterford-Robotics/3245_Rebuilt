package frc.robot;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.units.Units;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShootConstants;

public class Configs {

  public static final class ShootConfigs {

    // Shooter Kraken x60
    public static final TalonFXConfiguration SHOOT_TALON_FX_CONFIGURATION = new TalonFXConfiguration();

    // Shooter Indexer Kraken x60
    public static final TalonFXConfiguration SHOOTDEXER_TALON_FX_CONFIGURATION = new TalonFXConfiguration();

    // Conveyor CANRange
    public static final CANrangeConfiguration SHOOT_CANRANGE_CONFIGURATION = new CANrangeConfiguration();
    
    static {

      /*
       ********************************************
       **   SHOOTER KRAKEN x60 CONFIGURATIONS    **
       ********************************************
      */

      SHOOT_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = ShootConstants.k_shooterRampRate;
      SHOOT_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = ShootConstants.k_shooterClosedMaxSpeed;
      SHOOT_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -ShootConstants.k_shooterClosedMaxSpeed;
      SHOOT_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      SHOOT_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = ShootConstants.k_shooterSupplyCurrentLimit;

      SHOOTDEXER_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = ShootConstants.k_shooterRampRate;
      SHOOTDEXER_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = ShootConstants.k_shooterClosedMaxSpeed;
      SHOOTDEXER_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -ShootConstants.k_shooterClosedMaxSpeed;
      SHOOTDEXER_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      SHOOTDEXER_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = ShootConstants.k_shooterSupplyCurrentLimit;

      /*
       ********************************************
       **      SHOOT CANRANGE CONFIGURATION      **
       ********************************************
      */
      SHOOT_CANRANGE_CONFIGURATION.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
      SHOOT_CANRANGE_CONFIGURATION.ProximityParams.ProximityThreshold = 0.1;
      SHOOT_CANRANGE_CONFIGURATION.ProximityParams.ProximityHysteresis = 0.01; 
      SHOOT_CANRANGE_CONFIGURATION.FovParams.FOVRangeX = 6.75;
      SHOOT_CANRANGE_CONFIGURATION.FovParams.FOVRangeY = 6.75; 
      SHOOT_CANRANGE_CONFIGURATION.ToFParams.UpdateFrequency = 50;
      SHOOT_CANRANGE_CONFIGURATION.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;   
    }
  } 

  public static final class IntakeConfigs {

    // Intake Roller Kraken x44
    public static final TalonFXConfiguration INTAKE_ROLLER_TALON_FX_CONFIGURATION = new TalonFXConfiguration();

    // Intake Flipout Kraken x60
    public static final TalonFXConfiguration INTAKE_FLIPOUT_TALON_FX_CONFIGURATION = new TalonFXConfiguration();

    static{

      /*
       ********************************************
       **    INTAKE KRAKEN x44 CONFIGURATIONS    **
       ********************************************
      */

      INTAKE_ROLLER_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = IntakeConstants.k_intakeRampRate;
      INTAKE_ROLLER_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = IntakeConstants.k_intakeClosedMaxSpeed;
      INTAKE_ROLLER_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -IntakeConstants.k_intakeClosedMaxSpeed;
      INTAKE_ROLLER_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      INTAKE_ROLLER_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = IntakeConstants.k_intakeSupplyCurrentLimit;


      /*
       ********************************************
       **    INTAKE KRAKEN x60 CONFIGURATIONS    **
       ********************************************
      */
      INTAKE_FLIPOUT_TALON_FX_CONFIGURATION.Slot0.kP = IntakeConstants.k_intakeFlipoutP;
      INTAKE_FLIPOUT_TALON_FX_CONFIGURATION.Slot0.kI = IntakeConstants.k_intakeFlipoutI;
      INTAKE_FLIPOUT_TALON_FX_CONFIGURATION.Slot0.kD = IntakeConstants.k_intakeFlipoutD;
      INTAKE_FLIPOUT_TALON_FX_CONFIGURATION.Slot0.kS = IntakeConstants.k_intakeFlipoutS;
      INTAKE_FLIPOUT_TALON_FX_CONFIGURATION.Slot0.kV = IntakeConstants.k_intakeFlipoutV;
      INTAKE_FLIPOUT_TALON_FX_CONFIGURATION.Slot0.kA = IntakeConstants.k_intakeFlipoutA;
      INTAKE_FLIPOUT_TALON_FX_CONFIGURATION.Slot0.kG = IntakeConstants.k_intakeFlipoutG;

      INTAKE_FLIPOUT_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      INTAKE_FLIPOUT_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = 60;
      /*
      INTAKE_FLIPOUT_TALON_FX_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; 
      INTAKE_FLIPOUT_TALON_FX_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Rotations.of(0).in(Units.Rotations); // TODO: UPDATE FOR FINAL
      INTAKE_FLIPOUT_TALON_FX_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      INTAKE_FLIPOUT_TALON_FX_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Rotations.of(-3.2).in(Units.Rotations); // Starting position
      */
      INTAKE_FLIPOUT_TALON_FX_CONFIGURATION.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

      INTAKE_FLIPOUT_TALON_FX_CONFIGURATION.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
  }

  public static final class IndexConfigs {

    // Roller Floor Kraken x44
    public static final TalonFXConfiguration ROLLER_FLOOR_TALON_FX_CONFIGURATION = new TalonFXConfiguration();

    // Transfer Indexer Kraken x44
    public static final TalonFXConfiguration TRANSFER_INDEXER_TALON_FX_CONFIGURATION = new TalonFXConfiguration();

    static {
      
      /*
       ********************************************
       **    INDEXER KRAKEN x44 CONFIGURATIONS   **
       ********************************************
      */

      ROLLER_FLOOR_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = IndexerConstants.k_rollerIndexerRampRate;
      ROLLER_FLOOR_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = IndexerConstants.k_rollerIndexerClosedMaxSpeed;
      ROLLER_FLOOR_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -IndexerConstants.k_rollerIndexerClosedMaxSpeed;
      ROLLER_FLOOR_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ROLLER_FLOOR_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = IndexerConstants.k_rollerIndexerSupplyCurrentLimit;

      TRANSFER_INDEXER_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = IndexerConstants.k_rollerIndexerRampRate;
      TRANSFER_INDEXER_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = IndexerConstants.k_rollerIndexerClosedMaxSpeed;
      TRANSFER_INDEXER_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -IndexerConstants.k_rollerIndexerClosedMaxSpeed;
      TRANSFER_INDEXER_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      TRANSFER_INDEXER_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = IndexerConstants.k_rollerIndexerSupplyCurrentLimit;
    }
  }
}