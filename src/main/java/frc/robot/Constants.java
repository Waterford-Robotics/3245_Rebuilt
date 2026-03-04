// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

public final class Constants {

  // Drivebase
  public static final class DriveConstants {
    public static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  }

  // Controller Ports, Deadband, Buttons and Triggers
  public static final class ControllerConstants {

    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1; // Cure stick drift

    public static final int kStart = XboxController.Button.kStart.value;
    public static final int kBack = XboxController.Button.kBack.value;


    public static final int kA = XboxController.Button.kA.value;
    public static final int kB = XboxController.Button.kB.value;
    public static final int kX = XboxController.Button.kX.value;
    public static final int kY = XboxController.Button.kY.value;

    public static final int kDpadRight = 90; // D-Pad Right
    public static final int kDpadLeft = 270; // D-Pad Left

    public static final int k_rightbump = XboxController.Button.kRightBumper.value;
    public static final int k_leftbump = XboxController.Button.kLeftBumper.value;

    public static final int k_righttrig = XboxController.Axis.kRightTrigger.value;
    public static final int k_lefttrig = XboxController.Axis.kLeftTrigger.value;
  }

  // Auto stuff
  public static final class AutoConstants {

    // For Pathfinding TODO: Tune?
    public static final PathConstraints kconstraints = new PathConstraints(
      3.0, 
      4.0,
      Units.degreesToRadians(540), 
      Units.degreesToRadians(720)
    );
  }

  // Localization wow
  public static final class VisionConstants {
    // Name
    public static final String kLimelightFrontLeftName = "limelight-orange";
    public static final String kLimelightFrontRightName = "limelight-yellow";
    public static final String kLimelightBackLeftName = "limelight-green";
    public static final String kLimelightBackRightName = "limelight-blue";

    // Tag Reject Distance
    public static final double kRejectionDistance = 4;

    // Tag Reject Rotation Rate
    public static final int kRejectionRotationRate = 720;

    // Aim Controller PID Constants
    public static final double kPAim = 0.015;
    public static final double kIAim = 0.000;
    public static final double kDAim = 0.000;
  }

  public static final class PoseConstants {
    public static final Pose2d kRedTrenchLeftAlliance = new Pose2d(14, 2.5, Rotation2d.fromDegrees(180));  
  }

  // CAN IDs for Motors
  public static final class MotorIDConstants {

    public static final int k_shooterLeftID = 20;
    public static final int k_shooterRightID = 21;

    public static final int k_rollerIndexerID = 30;
    public static final int k_shooterIndexerID = 31;

    public static final int k_intakeRollerID = 40;
    public static final int k_innerIntakeID = 41;
  }

  // PWM IDs for Servos
  public static final class ServoConstants {
    public static final int k_servoID1 = 1;
    public static final int k_servoID2 = 2;
    
    //Subsystem
    public static final int k_servoSetpoint = 10;
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
    public static final double k_intakeSpeed = 0.5;
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

}
