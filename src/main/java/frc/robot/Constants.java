package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;

public class Constants {
    
  public static final class ShootConstants {
    public static final int k_shooter1ID = 22;
    public static final int k_shooter2ID = 23;
    public static final double k_shooterSpeed = -0.28;
    public static final double k_shooterRampRate = 0.05;
    public static final double k_shooterClosedMaxSpeed = 0.4;
    public static final int k_shooterSupplyCurrentLimit = 60;
  }

  public static final class IntakeConstants {
    //intake x40
    public static final int k_intakeID = 25;
    public static final int k_flipoutID = 22;
    public static final double k_intakeRampRate = 0.05;
    public static final double k_intakeClosedMaxSpeed = 0.4;
    public static final int k_intakeSupplyCurrentLimit = 60;
    public static final double k_intakeSpeed = -0.5;
  }

  public static final class ServoConstants {
    public static final int k_servoID1 = 1;
    public static final int k_servoID2 = 2;
  }

  public static final class SensorIDConstants {
    // Intake CANRange
    public static final int k_shootCANRangeID = 42;
  }

  // Controller Ports, Deadband, Buttons and Triggers
  public static final class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;

    public static final int kStart = XboxController.Button.kStart.value;

    public static final int kA = XboxController.Button.kA.value;
    public static final int kB = XboxController.Button.kB.value;
    public static final int kX = XboxController.Button.kX.value;
    public static final int kY = XboxController.Button.kY.value;

    public static final int kDpadRight = 90; // D-Pad Right
    public static final int kDpadLeft = 270; // D-Pad Left

    public final static int k_righttrig = Axis.kRightTrigger.value; // Right Trig
    public final static int k_lefttrig = Axis.kLeftTrigger.value; // Left Trig
  }

    public static final class VisionConstants {
    // Name
    public static final String kLimelightName = "limelight-two";

    // PID for Tag Relative Control in General
    public static final double kPAim = 0.015;
    public static final double kIAim = 0.000;
    public static final double kDAim = 0.000;

  }  
}
