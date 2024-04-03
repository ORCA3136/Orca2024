package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.choreo.lib.Choreo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.commands.NoteOffIntake;
import frc.robot.commands.SpeakerCentering;

public class Trajectories {
    
    private PIDController AutoDrivePID = new PIDController(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD);
    private PIDController AutoTurnPID = new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD);

    private DriveSubsystem m_robotDrive;
    private ShooterSubsystem m_ShooterSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;
    private ArmSubsystem m_ArmSubsystem;
    private SensorSubsystem m_SensorSubsystem;
    private XboxController m_DriverController;

    Trajectory[] blueTrajectories;
    Trajectory[] redTrajectories;

    Command blueDoubleSpeaker;

    TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(false);

    TrajectoryConfig slowConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared / 2)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(false);

    public Trajectories(DriveSubsystem drive) {
        m_robotDrive = drive;

        blueTrajectories = CreateTrajectories(false);
        redTrajectories = CreateTrajectories(true);
    }

    public Command DriveTrajectory(String Trajectory) {

        Optional<Alliance> RobotAlliance;
        RobotAlliance = DriverStation.getAlliance();

        return Choreo.choreoSwerveCommand(
            Choreo.getTrajectory("DriveForward"), 
            () -> (m_robotDrive.getPose()), 
            AutoDrivePID, AutoDrivePID, AutoTurnPID, 
            (ChassisSpeeds speeds) -> m_robotDrive.driveRobotRelative(speeds),
            () -> RobotAlliance.get() == Alliance.Red, 
            m_robotDrive
            );
    }

    public Trajectory[] CreateTrajectories(boolean isRed) {
        
        double b;
        if (isRed) b = -1;
        else b = 1;

        Pose2d speakerPose = new Pose2d(b * -6.8, 1.295, new Rotation2d(calcAngle(0, isRed)));
        Pose2d sideSpeakerPose = new Pose2d(b * -7.42, 0.3, new Rotation2d(calcAngle(-Math.PI/4, isRed)));
        Pose2d ampPose = new Pose2d(b * -6.45, 3.63, new Rotation2d(calcAngle(-Math.PI/2, isRed)));
        Pose2d ampNotePose = new Pose2d(b * -5.57, 3.2, new Rotation2d(calcAngle(0.2, isRed)));
        Pose2d middleNotePose = new Pose2d(b * -5.67, 1.295, new Rotation2d(calcAngle(0, isRed)));
        Pose2d sourceNotePose = new Pose2d(b * -5.95, 0, new Rotation2d(calcAngle(0, isRed)));
        Pose2d ampCenterShotPose = new Pose2d(b * -3.72, 2.53, new Rotation2d(calcAngle(0.15, isRed)));
        Pose2d sourceCenterShotPose = new Pose2d(b * -3.72, 2.53, new Rotation2d(calcAngle(-0.4, isRed)));
        
        // Trajectories
        // Speaker -> first note
        Trajectory driveForward = TrajectoryGenerator.generateTrajectory(
            speakerPose,
            List.of(new Translation2d(b * -6.27, 1.395)),
            middleNotePose,
            config);

        // Corner of starting zone -> Amp
        Trajectory driveToAmp = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -6.82, 2.9, new Rotation2d(calcAngle(-Math.PI/2, isRed))),
            List.of(new Translation2d(b * -6.4, 3.395)),
            ampPose,
            config);

        // Amp -> amp note
        Trajectory driveToAmpNote = TrajectoryGenerator.generateTrajectory(
            ampPose,
            List.of(new Translation2d(b * -6.27, 3.395)),
            ampNotePose,
            config);

        Trajectory driveAlongAmpWall = TrajectoryGenerator.generateTrajectory(
            ampPose,
            List.of(new Translation2d(b * -4.7, 3.35)),
            new Pose2d(b * -3.5, 3.3, new Rotation2d(calcAngle(-Math.PI/2, isRed))),
            slowConfig);

        Trajectory ampWallToLeftCenterNote = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -2.77, 3.4, new Rotation2d(calcAngle(-Math.PI/2, isRed))),
            List.of(),
            new Pose2d(b * -0.6, 3.4, new Rotation2d(calcAngle(0, isRed))),
            config);

        Trajectory leftCenterNoteToShot = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -0.4, 3.4, new Rotation2d(calcAngle(0, isRed))),
            List.of(),
            ampCenterShotPose,
            config);

        // Amp note -> amp
        Trajectory ampFromNote = TrajectoryGenerator.generateTrajectory(
            ampNotePose,
            List.of(new Translation2d(b * -6.27, 3.395)),
            ampPose,
            config);

        // Side of speaker -> source side
        Trajectory driveSourceCenterPickup = TrajectoryGenerator.generateTrajectory(
            sideSpeakerPose,
            List.of(new Translation2d(b * -4, -2.2)),
            new Pose2d(b * -0.6, -3, new Rotation2d(calcAngle(0, isRed))),
            config);

        Trajectory sourceCenterNoteToShoot = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -0.6, -3, new Rotation2d(calcAngle(0, isRed))),
            List.of(new Translation2d(b * -2.5, -1.3)),
            sourceCenterShotPose,
            config);

        // Speaker -> source note
        Trajectory speakerToSourceNote = TrajectoryGenerator.generateTrajectory(
            speakerPose,
            List.of(),
            sourceNotePose,
            config);

        Trajectory[] trajectories = new Trajectory[] {driveForward, driveToAmp, driveToAmpNote, 
            driveAlongAmpWall, ampWallToLeftCenterNote, leftCenterNoteToShot,
            driveSourceCenterPickup, sourceCenterNoteToShoot
            };
        return trajectories;
    }

    private double calcAngle(double angle, Boolean isRed) {
        if (isRed) {
            if (angle > 0) angle -= Math.PI;
            else angle += Math.PI;
            angle *= -1;
        }

        return angle;
    }

    public Trajectory PathFromCurrentPose(Pose2d targetPose) {
                
        return TrajectoryGenerator.generateTrajectory(
            m_robotDrive.getPose(),
            List.of(),
            targetPose,
            config);
    }

    public Trajectory MultiplePathFromCurrentPose(Translation2d[] listOfPoses, Pose2d targetPose) {
        return TrajectoryGenerator.generateTrajectory(
            m_robotDrive.getPose(),
            List.of(listOfPoses),
            targetPose,
            config);
    }
      
    public Trajectory[] GetTrajectories(boolean isRed) {
        if (isRed) return redTrajectories;
        return blueTrajectories;
    }


    // Half x = 8.27
    // Half y = 4.105
    // Speaker Pose2d(-6.82, 1.295, 0 or Math.PI)   Speaker Pose2d(1.45, 5.4, 0 or Math.PI)
    // 
}