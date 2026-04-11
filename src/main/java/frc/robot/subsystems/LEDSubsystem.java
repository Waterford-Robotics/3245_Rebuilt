package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  Spark m_blinkin;
  // ColorSensorSubsystem m_sensor = new ColorSensorSubsystem();

  public LEDSubsystem() {
    //enter port constant
    m_blinkin = new Spark(LEDConstants.k_ledID);
  } 
  //colors

  public void setRainbowParty(){
    m_blinkin.set(-0.97);
  }

  public void setRed() {
    m_blinkin.set(0.61);
  }

  public void setOrange() {
    m_blinkin.set(0.65);
  }

  public void setYellow() {
    m_blinkin.set(0.69);
  }

  public void setGreen() {
    m_blinkin.set(0.77);
  }

  public void setBlue() {
    m_blinkin.set(0.87);
  }

  public void setPurple() {
    m_blinkin.set(0.91);
  }

  public void setFireMedium() {
    m_blinkin.set(-0.59);
  }

  public void turnOff() {
    m_blinkin.set(0.00);
  }
  
  public void setTwinklesLavaParty() {
    m_blinkin.set(-0.49);
  }
  
  public void setTwinklesOceanParty() {
  m_blinkin.set(-0.51);
  }
}