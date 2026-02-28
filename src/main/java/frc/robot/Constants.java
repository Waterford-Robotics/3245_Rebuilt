package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

public class Constants {

  // Controller Ports, Deadband, Buttons and Triggers
  public static final class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;

    public static final int kStart = XboxController.Button.kStart.value;

    public static final int kA = XboxController.Button.kA.value;
    public static final int kB = XboxController.Button.kB.value;
    public static final int kX = XboxController.Button.kX.value;
    public static final int kY = XboxController.Button.kY.value;

    public final static int k_rightbump = Button.kRightBumper.value; // Right Bump
    public final static int k_leftbump = Button.kLeftBumper.value; // Left Bump

    public static final int kDpadRight = 90; // D-Pad Right
    public static final int kDpadLeft = 270; // D-Pad Left

    public final static int k_righttrig = Axis.kRightTrigger.value; // Right Trig
    public final static int k_lefttrig = Axis.kLeftTrigger.value; // Left Trig
  }

  // CAN IDs for Motors
  public static final class MotorIDConstants {

    public static final int k_shooterRightID = 21;
    public static final int k_shooterLeftID = 22;

    public static final int k_rollerIndexerID = 30;
    public static final int k_shooterIndexerID = 31;

    public static final int k_intakeRollerID = 40;
    public static final int k_innerIntakeID = 41;
  }

  // PWM IDs for Servos
  public static final class ServoConstants {
    public static final int k_servoID1 = 1;
    public static final int k_servoID2 = 2;
  }

  // CAN IDs for Sensors
  public static final class SensorIDConstants {
    public static final int k_shootCANRangeID = 50;
  }
  
  // Shooter Stuff
  public static final class ShootConstants {
    
    // Configs
    public static final double k_shooterRampRate = 0.05;
    public static final double k_shooterClosedMaxSpeed = 0.9;
    public static final int k_shooterSupplyCurrentLimit = 60;

    // Subsystem
    public static final double k_shooterSpeed = 0.35;
  }

  // Intake Stuff
  public static final class IntakeConstants {

    // Configs
    public static final double k_intakeRampRate = 0.05;
    public static final double k_intakeClosedMaxSpeed = 0.4;
    public static final int k_intakeSupplyCurrentLimit = 60;

    // Subsystems
    public static final double k_intakeSpeed = -0.5;
  }

  // Indexer Stuff 
  public static final class IndexerConstants {

    // Configs
    public static final double k_rollerIndexerRampRate = 0.05;
    public static final double k_rollerIndexerClosedMaxSpeed = 0.4;
    public static final int k_rollerIndexerSupplyCurrentLimit = 40;

    // Subsystems
    public static final double k_indexerSpeed = 0.25;
  }

  // Limelight Stuff
  public static final class VisionConstants {

    // PID for Tag Relative Control in General
    public static final double kPAim = 0.015;
    public static final double kIAim = 0.000;
    public static final double kDAim = 0.000;

  }  
}
