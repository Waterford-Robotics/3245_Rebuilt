
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
        
    public void climbUp() {
        m_climber_1.set(ClimberConstants.k_climbUpSpeed);
        m_climber_2.set(ClimberConstants.k_climbUpSpeed);
    }

    public void stopClimb() {
        m_climber_1.set(0);
        m_climber_2.set(0);
    }

    public void climbDown() {
        m_climber_1.set(ClimberConstants.k_climbDownSpeed);
        m_climber_2.set(ClimberConstants.k_climbDownSpeed);
    }

    public void periodic() {
    }
}
