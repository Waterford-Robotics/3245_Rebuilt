package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  Spark m_blinkin;
  // ColorSensorSubsystem m_sensor = new ColorSensorSubsystem();

  public LEDSubsystem() {
    //enter port constant
    m_blinkin = new Spark(1);
  } 
  //colors

  public void setStrobeGold(){
    m_blinkin.set(-0.07);
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
    
  public void setTeamColor() {
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()) {
      if(alliance.get() == DriverStation.Alliance.Red) {
          setRed();
      }
      else if(alliance.get() == DriverStation.Alliance.Blue) {
          setBlue();
      }
    }
  }
    
  public void setTeamColorTwinkles() {
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      if(alliance.get() == DriverStation.Alliance.Red) {
          setTwinklesLavaParty();
      }
      else if(alliance.get() == DriverStation.Alliance.Blue) {
          setTwinklesOceanParty();
      }
    }
  }
}