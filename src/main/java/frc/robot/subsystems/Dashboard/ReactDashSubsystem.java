package frc.robot.subsystems.Dashboard;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReactDashSubsystem extends SubsystemBase {

  public static final NetworkTable ReactDash = NetworkTableInstance.getDefault().getTable("ReactDash");

  private StringPublisher m_goTotabPub;

  private IntegerPublisher m_matchTimePub;

  public ReactDashSubsystem() {
    var autoTable = ReactDashSubsystem.ReactDash.getSubTable("Main");
    m_goTotabPub = autoTable.getStringTopic("rpub/goTotab").publish();
    m_matchTimePub = autoTable.getIntegerTopic("rpub/matchTime").publish();
  }

  public void periodic() {
    m_matchTimePub.set((int) DriverStation.getMatchTime());
  }

  public void SwitchTab(String tab) {
    m_goTotabPub.set(tab);
  }
}