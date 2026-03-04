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

public final class Constants {

  // Drivebase
  public static final class DriveConstants {
    public static final double k_maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double k_maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  }

  // Controller Ports, Deadband, Buttons and Triggers
  public static final class ControllerConstants {

    public static final int k_driverControllerPort = 0;
    public static final double k_driveDeadband = 0.1; // Cure stick drift

    public static final int k_start = XboxController.Button.kStart.value;
    public static final int k_back = XboxController.Button.kBack.value;

    public static final int k_A = XboxController.Button.kA.value;
    public static final int k_B = XboxController.Button.kB.value;
    public static final int k_X = XboxController.Button.kX.value;
    public static final int k_Y = XboxController.Button.kY.value;

    public static final int k_dpadRight = 90; // D-Pad Right
    public static final int k_dpadLeft = 270; // D-Pad Left
  }

  // Auto stuff
  public static final class AutoConstants {

    // For Pathfinding TODO: Tune?
    public static final PathConstraints k_constraints = new PathConstraints(
      3.0, 
      4.0,
      Units.degreesToRadians(540), 
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
  }

  // Localization code stuff omg
  public static final class PoseConstants {

    // RED TRENCH LEFT
    public static final Pose2d k_redTrenchLeftNeutralPose = new Pose2d(10.25, 0.6, Rotation2d.fromDegrees(180));  
    public static final Pose2d k_redTrenchLeftAlliancePose = new Pose2d(13.5, 0.6, Rotation2d.fromDegrees(180));
    
    // RED TRENCH RIGHT
    public static final Pose2d k_redTrenchRightNeutralPose = new Pose2d(10.25, 6.5, Rotation2d.fromDegrees(180));  
    public static final Pose2d k_redTrenchRightAlliancePose = new Pose2d(13.5, 6.5, Rotation2d.fromDegrees(180));

    // BLUE TRENCH LEFT
    public static final Pose2d k_blueTrenchLeftNeutralPose = new Pose2d(6, 6.5, Rotation2d.fromDegrees(180));  
    public static final Pose2d k_blueTrenchLeftAlliancePose = new Pose2d(3, 6.5, Rotation2d.fromDegrees(180));

    // BLUE TRENCH RIGHT
    public static final Pose2d k_blueTrenchRightNeutralPose = new Pose2d(6, 0.6, Rotation2d.fromDegrees(180));  
    public static final Pose2d k_blueTrenchRightAlliancePose = new Pose2d(3, 0.6, Rotation2d.fromDegrees(180));
  }
}