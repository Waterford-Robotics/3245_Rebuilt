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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.generated.TunerConstants;

public final class Constants {

  // Drivebase
  public static final class DriveConstants {
    public static final double k_maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double k_maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  }
  

  // Controller Ports, Deadband, Buttons and Triggers
  public static final class ControllerConstants {

    public static final int k_driverControllerPort = 0;
    public static final int k_operatorControllerPort = 1;

    public static final double k_driveDeadband = 0.1; // Cure stick drift

    public static final int k_start = XboxController.Button.kStart.value; // Start
    public static final int k_back = XboxController.Button.kBack.value; // Back


    public static final int k_A = XboxController.Button.kA.value; // A
    public static final int k_B = XboxController.Button.kB.value; // B
    public static final int k_X = XboxController.Button.kX.value; // X
    public static final int k_Y = XboxController.Button.kY.value; // Y

    public static final int k_dpadUp = 0; // D-Pad Up
    public static final int k_dpadRight = 90; // D-Pad Right
    public static final int k_dpadDown = 180; // D-Pad Right
    public static final int k_dpadLeft = 270; // D-Pad Left

    public static final int k_rightbump = XboxController.Button.kRightBumper.value; // Right Bump
    public static final int k_leftbump = XboxController.Button.kLeftBumper.value; // Left Bump

    public static final int k_righttrig = XboxController.Axis.kRightTrigger.value; // Right Trigger
    public static final int k_lefttrig = XboxController.Axis.kLeftTrigger.value; // Left Trigger
  }

  // Auto stuff
  public static final class AutoConstants {

    // For General Pathfinding
    public static final PathConstraints k_defaultConstraints = new PathConstraints(
      3.0, 
      4.0,
      Units.degreesToRadians(540), 
      Units.degreesToRadians(720)
    );

    // For Pathfinding with Intake
    public static final PathConstraints k_intakingConstraints = new PathConstraints(
      3.0, 
      3.0,
      Units.degreesToRadians(90), 
      Units.degreesToRadians(720)
    );
  }

  // Localization wow
  public static final class VisionConstants {

    // Name
    public static final String k_limelightFrontLeftName = "limelight-orange";
    public static final String k_limelightFrontRightName = "limelight-yellow";
    public static final String k_limelightBackLeftName = "limelight-green";
    public static final String k_limelightBackRightName = "limelight-blue";

    // Tag Reject Distance
    public static final double k_rejectionDistance = 4;

    // Tag Reject Rotation Rate
    public static final int k_rejectionRotationRate = 720;

    // Aim Controller PID Constants
    public static final double kPAim = 0.015;
    public static final double kIAim = 0.000;
    public static final double kDAim = 0.000;
  }

  //s
  public static final class LEDConstants {
    public static final int k_ledID = 0;
  }
  
  // Localization code stuff omg
  public static final class PoseConstants {

    // RED START
    public static final Pose2d k_redLeftAutoStartingPose = new Pose2d(12.703, 0.394, Rotation2d.fromDegrees(103.6));

    // RED TRENCH LEFT
    public static final Pose2d k_redTrenchLeftFarPose = new Pose2d(8.7, 0.6, Rotation2d.fromDegrees(180));
    public static final Pose2d k_redTrenchLeftNeutralPose = new Pose2d(10.25, 0.6, Rotation2d.fromDegrees(180)); 
    public static final Pose2d k_redTrenchLeftAlliancePose = new Pose2d(13.5, 0.6, Rotation2d.fromDegrees(180));

    public static final Pose2d k_redTrenchLeftFarPoseRotated = new Pose2d(9.1, 0.6, Rotation2d.fromDegrees(90));
    public static final Pose2d k_redTrenchLeftNeutralPoseRotated = new Pose2d(10.25, 0.6, Rotation2d.fromDegrees(90)); 
    public static final Pose2d k_redTrenchLeftAlliancePoseRotated = new Pose2d(13.5, 0.6, Rotation2d.fromDegrees(90));  

    // RED TRENCH LEFT COLLECTION POSES
    public static final Pose2d k_redLeftIntakeInitiatePose = new Pose2d(8.5, 1, Rotation2d.fromDegrees(90));   
    public static final Pose2d k_redLeftIntakeTerminatePose = new Pose2d(8.5, 1, Rotation2d.fromDegrees(90)); 

    // RED TRENCH LEFT SCORE POSE
    public static final Pose2d k_redLeftShootPose = new Pose2d(14, 2, Rotation2d.fromDegrees(135)); 

    // RED TRENCH RIGHT
    public static final Pose2d k_redTrenchRightNeutralPose = new Pose2d(10.25, 6.5, Rotation2d.fromDegrees(180));  
    public static final Pose2d k_redTrenchRightNeutralPoseRotated = new Pose2d(10.25, 6.5, Rotation2d.fromDegrees(90));
    public static final Pose2d k_redTrenchRightAlliancePose = new Pose2d(13.5, 6.5, Rotation2d.fromDegrees(180));

    // BLUE TRENCH LEFT
    public static final Pose2d k_blueTrenchLeftNeutralPose = new Pose2d(6, 6.5, Rotation2d.fromDegrees(180));  
    public static final Pose2d k_blueTrenchLeftAlliancePose = new Pose2d(3, 6.5, Rotation2d.fromDegrees(180));

    // BLUE TRENCH RIGHT
    public static final Pose2d k_blueTrenchRightNeutralPose = new Pose2d(6, 0.6, Rotation2d.fromDegrees(180));  
    public static final Pose2d k_blueTrenchRightAlliancePose = new Pose2d(3, 0.6, Rotation2d.fromDegrees(180));
  }

  // CAN IDs for Motors
  public static final class MotorIDConstants {

    // Shooter
    public static final int k_shooterLeftID = 20;
    public static final int k_shooterRightID = 21;

    // Indexer
    public static final int k_rollerFloorID = 30;
    public static final int k_transferIndexerID = 31;

    // Intake
    public static final int k_intakeRollerID = 40;
    public static final int k_intakeFlipoutLeftID = 41;
    public static final int k_intakeFlipoutRightID = 42;
  }

  // Servo Stuff
  public static final class ServoConstants {

    // PWM IDs for Servos
    public static final int k_servoID1 = 1;
    public static final int k_servoID2 = 2;
    
    // Setpoints
    public static final int k_servoSetpoint = 10;
  }

  // CAN IDs for Sensors
  public static final class SensorIDConstants {

    // Shooter CANRange
    public static final int k_shootCANRangeID = 50;

    // Intake CANCoder TODO: REMOVE?
    public static final int k_intakeCANCoderID = 51;
  }
  
  // Shooter Stuff
  public static final class ShootConstants {
    
    // Configs
    public static final double k_shooterRampRate = 0.05;
    public static final double k_shooterClosedMaxSpeed = 0.95;
    public static final int k_shooterSupplyCurrentLimit = 60;

    // Subsystem
    public static final double k_hubShotSpeed = 0.42; // 0.42
    public static final double k_autoHubShotSpeed = 0.65; 
    public static final double k_maxShootRPS = 100.0;
  }

  // Intake Stuff
  public static final class IntakeConstants {

    // Configs
    public static final double k_intakeRampRate = 0.05;
    public static final double k_intakeClosedMaxSpeed = 0.95;
    public static final int k_intakeSupplyCurrentLimit = 40;

    // Subsystems
    public static final double k_intakeSpeed = 0.40;

    // Intake Deploy Angles
    public static final Angle k_intakeRetractedAngle = edu.wpi.first.units.Units.Rotations.of(-3.3);
    public static final Angle k_intakedexRetractedAngle = edu.wpi.first.units.Units.Rotations.of(-2.0);
    public static final Angle k_intakedexDeployedAngle = edu.wpi.first.units.Units.Rotations.of(-1.0);
    public static final Angle k_intakeDeployedAngle = edu.wpi.first.units.Units.Rotations.of(0);

    // Flipout PIDs TODO: RETUNE ME WHEN ASSEMBLED
    public static final double k_intakeFlipoutP = 2.5; 
    public static final double k_intakeFlipoutI = 0;
    public static final double k_intakeFlipoutD = 0.3;
    public static final double k_intakeFlipoutS = 0.20;
    public static final double k_intakeFlipoutV = 1.20;
    public static final double k_intakeFlipoutA = 0.0;
    public static final double k_intakeFlipoutG = 0.5;
  }

  // Indexer Stuff 
  public static final class IndexerConstants {

    // Configs
    public static final double k_rollerIndexerRampRate = 0.05;
    public static final double k_rollerIndexerClosedMaxSpeed = 0.95;
    public static final int k_rollerIndexerSupplyCurrentLimit = 40;

    // Subsystems
    public static final double k_indexerSpeed = 0.50;
  }

  public static final class LiveConstants {
    public static boolean _enableShooter = false;
  }
}
