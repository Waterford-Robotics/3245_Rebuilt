package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {
    Servo m_linearServo;
    double m_speed;
	double m_length;
	double setPos;
	double curPos;
	double setPoint;

    public ServoSubsystem(int channel, int length, int speed){
        m_linearServo = new Servo(channel);
        m_linearServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
		m_length = length;
		m_speed = speed;
		SmartDashboard.putNumber("Servo Setpoint", 10);
    }
    /**
	 * Run this method in any periodic function to update the position estimation of
	 * your
	 * servo
	 *
	 * @param setpoint the target position of the servo [mm]
	 */
	public void setPosition(double setpoint) {
		setPos = MathUtil.clamp(setpoint, 0, m_length);
		m_linearServo.setSpeed((setPos / m_length * 2) - 1);
		SmartDashboard.putNumber("Linear Servo Setpoint " + m_linearServo.getChannel(), (setPos / m_length * 2) - 1);
	}

	double lastTime = 0;

	/**
	 * Run this method in any periodic function to update the position estimation of
	 * your
	 * servo
	 */
	public void updateCurPos() {
		double dt = Timer.getFPGATimestamp() - lastTime;
		if (curPos > setPos + m_speed * dt) {
			curPos -= m_speed * dt;
		} else if (curPos < setPos - m_speed * dt) {
			curPos += m_speed * dt;
		} else {
			curPos = setPos;
		}
	}

	/**
	 * Current position of the servo, must be calling {@link #updateCurPos()
	 * updateCurPos()} periodically
	 *
	 * @return Servo Position [mm]
	 */
	public double getPosition() {
		return curPos;
	}

	/**
	 * Checks if the servo is at its target position, must be calling
	 * {@link #updateCurPos()
	 * updateCurPos()} periodically
	 *
	 * @return true when servo is at its target
	 */
	public boolean isFinished() {
		return curPos == setPos;
	}

	public double getSetpoint() {
		setPoint = SmartDashboard.getNumber("Servo Setpoint", 10);
		return setPoint;
	}

}
