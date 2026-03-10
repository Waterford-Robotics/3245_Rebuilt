package frc.robot;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CANRangeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ServoSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

import frc.robot.commands.ShootForSecsCommand;
import frc.robot.commands.AssistedShootForSecsCommand;
import frc.robot.commands.RunIntakeForSecsCommand;
import frc.robot.commands.AutoIndexCommand;
import frc.robot.commands.IndexForSecsCommand;
import frc.robot.commands.PathfindToPoseCommand;

import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.AutoConstants;

public class AutoFactory {

  private CommandSwerveDrivetrain m_drivetrain;
  private IntakeSubsystem m_intakeSubsystem;
  private IndexerSubsystem m_indexerSubsystem;
  private CANRangeSubsystem m_canRangeSubsystem;
  private ServoSubsystem m_servoSubsystem1;
  private ServoSubsystem m_servoSubsystem2;
  private ShootSubsystem m_shootSubsystem;

  public AutoFactory(
    CommandSwerveDrivetrain drivetrain, 
    IntakeSubsystem intake, 
    IndexerSubsystem indexer, 
    CANRangeSubsystem canRange,
    ServoSubsystem servo1,
    ServoSubsystem servo2,
    ShootSubsystem shooter
    ) {

      this.m_drivetrain = drivetrain;
      this.m_intakeSubsystem = intake;
      this.m_indexerSubsystem = indexer;
      this.m_canRangeSubsystem = canRange;
      this.m_servoSubsystem1 = servo1;
      this.m_servoSubsystem2 = servo2;
      this.m_shootSubsystem = shooter;
  }

  // Hub Shot Auto - Starts Directly in Front of Hub, Shoots Balls x8
  public SequentialCommandGroup HubShotAuto() {
    return new SequentialCommandGroup(
      // Spin Up
      new ParallelDeadlineGroup(
        new ShootForSecsCommand(this.m_shootSubsystem, 2), // TODO: Condense to one command
        // new RunIntakeForSecsCommand(this.m_intakeSubsystem, 5),
        new AutoIndexCommand(this.m_indexerSubsystem, m_canRangeSubsystem)
      ),
      // Shoot
      new ParallelDeadlineGroup(
        new ShootForSecsCommand(m_shootSubsystem, 4), 
        // new RunIntakeForSecsCommand(m_intakeSubsystem, 5),
        new IndexForSecsCommand(this.m_indexerSubsystem, 5)
      )
    );  
  }

  // Red Alliance Left Trench Auto - Starts on the line in Left Trench of Red Alliance, Shoots Balls x8, Collects, Shoot Again
  public SequentialCommandGroup RedLeftTrenchSingleDipAuto() {
    return new SequentialCommandGroup(
      // Spin Up
      new ParallelDeadlineGroup(
        new AssistedShootForSecsCommand(m_servoSubsystem1, m_servoSubsystem2, m_shootSubsystem, 2),
        new AutoIndexCommand(this.m_indexerSubsystem, m_canRangeSubsystem)
      ),
      // Shoot
      new ParallelDeadlineGroup(
        new AssistedShootForSecsCommand(m_servoSubsystem1, m_servoSubsystem2, m_shootSubsystem, 2),
        new IndexForSecsCommand(this.m_indexerSubsystem, 5)
      ),
      // Intake
      new ParallelDeadlineGroup(
        new WaitCommand(3),
        new PathfindToPoseCommand(this.m_drivetrain, PoseConstants.k_redTrenchLeftNeutralPoseRotated, AutoConstants.k_constraints)
      ),
      // new PathfindToPoseCommand(this.m_drivetrain, PoseConstants.k_redLeftIntakeInitiatePose, AutoConstants.k_constraints),
      new ParallelDeadlineGroup(
        new PathfindToPoseCommand(this.m_drivetrain, PoseConstants.k_redLeftIntakeInitiatePose, AutoConstants.k_constraints),
        new RunIntakeForSecsCommand(this.m_intakeSubsystem, 5)
      ),
      // Return
      new PathfindToPoseCommand(this.m_drivetrain, PoseConstants.k_redTrenchLeftNeutralPose, AutoConstants.k_constraints),
      new PathfindToPoseCommand(this.m_drivetrain, PoseConstants.k_redTrenchLeftAlliancePose, AutoConstants.k_constraints),
      new PathfindToPoseCommand(this.m_drivetrain, PoseConstants.k_redLeftShootPose, AutoConstants.k_constraints),

      // Spin Up TODO: RUN IN Parallel with Return stuff
      new ParallelDeadlineGroup(
        new ShootForSecsCommand(this.m_shootSubsystem, 2), 
        new RunIntakeForSecsCommand(this.m_intakeSubsystem, 5),
        new AutoIndexCommand(this.m_indexerSubsystem, m_canRangeSubsystem)
      ),
      // Shoot
      new ParallelDeadlineGroup(
        new AssistedShootForSecsCommand(m_servoSubsystem1, m_servoSubsystem2, m_shootSubsystem, 10),
        new RunIntakeForSecsCommand(m_intakeSubsystem, 5),
        new IndexForSecsCommand(this.m_indexerSubsystem, 5)
      )
    );
  }
}