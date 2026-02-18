
public class ClimbSubsystem extends SubsystemBase {
    private TalonFX m_climber_1;
    private TalonFX m_climber_2;
    private TalonFXConfiguration shooterConfig;

    public ClimbSubsystem() {
        // m_shooter = new TalonFX(MotorIDConstants.k_shooterKrakenID, "Elevator/Coral");
        // shooterConfig = new TalonFXConfiguration();


        // // Kraken Configs
        // shooterConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = WristConstants.k_shooterRampRate;
        // shooterConfig.MotorOutput.PeakForwardDutyCycle = WristConstants.k_shooterClosedMaxSpeed;
        // shooterConfig.MotorOutput.PeakReverseDutyCycle = -WristConstants.k_shooterClosedMaxSpeed;
        // shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // shooterConfig.CurrentLimits.SupplyCurrentLimit = WristConstants.k_intakeSupplyCurrentLimit;

        // m_shooter.getConfigurator().apply(shooterConfig, 0.05);
    }
    public void windup() {
        m_shooter.set(ClimberConstants.k_climbSpeed);
    }

    public void stopWindup() {
        m_shooter.set(0);
    }

    public void intake() {
        m_shooter.set(/*-soome value*/);
    }

    public void periodic() {
    }
}
