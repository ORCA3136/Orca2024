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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class Trajectories {
    
    private PIDController AutoDrivePID = new PIDController(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD);
    private PIDController AutoTurnPID = new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD);

    private DriveSubsystem robotDrive;

    private ArrayList<Trajectory> middleDoubleScore = new ArrayList<Trajectory>();
    private ArrayList<Trajectory> tripleSpeakerScore = new ArrayList<Trajectory>();
    private ArrayList<Trajectory> ampDoubleScore = new ArrayList<Trajectory>();
    private ArrayList<Trajectory> driveForwardAuto = new ArrayList<Trajectory>();

    TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(false);

    public Trajectories(DriveSubsystem drive) {
        robotDrive = drive;
    }

    public Command DriveTrajectory(String Trajectory) {

        Optional<Alliance> RobotAlliance;
        RobotAlliance = DriverStation.getAlliance();

        return Choreo.choreoSwerveCommand(
            Choreo.getTrajectory("DriveForward"), 
            () -> (robotDrive.getPose()), 
            AutoDrivePID, AutoDrivePID, AutoTurnPID, 
            (ChassisSpeeds speeds) -> robotDrive.driveRobotRelative(speeds),
            () -> RobotAlliance.get() == Alliance.Red, 
            robotDrive
            );
    }

    public ArrayList<Trajectory> getMiddleDoubleScore() {
        return middleDoubleScore;
    }

    public ArrayList<Trajectory> getTripleSpeakerScore() {
        return tripleSpeakerScore;
    }

    public ArrayList<Trajectory> getAmpDoubleScore() {
        return ampDoubleScore;
    }

    public ArrayList<Trajectory> getDriveForward() {
        return driveForwardAuto;
    }

    public void CreateTrajectories(boolean red) {

        int j = middleDoubleScore.size();
        for (int i = 0; i < j; i ++) {
            middleDoubleScore.remove(0);
        }
        j = tripleSpeakerScore.size();
        for (int i = 0; i < j; i ++) {
            tripleSpeakerScore.remove(0);
        }
        j = ampDoubleScore.size();
        for (int i = 0; i < j; i ++) {
            ampDoubleScore.remove(0);
        }
        j = driveForwardAuto.size();
        for (int i = 0; i < j; i ++) {
            driveForwardAuto.remove(0);
        }

        boolean isRed = red;

        double b = 1;

        if (isRed) {
            b = -1;
        }
        else {
            b = 1;
        }

        Pose2d speakerPose = new Pose2d(b * -6.65, 1.295, new Rotation2d(calcAngle(0, isRed)));
        Pose2d sideSpeakerPose = new Pose2d(b * -7.42, 0.13, new Rotation2d(calcAngle(0, isRed)));
        Pose2d ampPose = new Pose2d(b * -6.52, 3.65, new Rotation2d(calcAngle(-Math.PI/2, isRed)));
        Pose2d ampNotePose = new Pose2d(b * -5.57, 2.945, new Rotation2d(calcAngle(0, isRed)));
        Pose2d middleNotePose = new Pose2d(b * -5.67, 1.295, new Rotation2d(calcAngle(0, isRed)));
        Pose2d sourceNotePose = new Pose2d(b * -5.95, 0, new Rotation2d(calcAngle(0, isRed)));
        
        // Trajectories
        // Speaker -> first note
        Trajectory driveForward = TrajectoryGenerator.generateTrajectory(
            speakerPose,
            List.of(new Translation2d(b * -6.27, 1.395)),
            middleNotePose,
            config);

        // First note -> speaker
        Trajectory driveBackward = TrajectoryGenerator.generateTrajectory(
            middleNotePose,
            List.of(new Translation2d(b * -6.27, 1.195)),
            speakerPose,
            config);

        // Speaker -> past first note
        Trajectory driveFartherForward = TrajectoryGenerator.generateTrajectory(
            speakerPose,
            List.of(new Translation2d(b * -5.87, 1.395)),
            new Pose2d(b * -4.67, 1.295, new Rotation2d(calcAngle(0, isRed))),
            config);

        // Corner of starting zone -> Amp
        Trajectory driveToAmp = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -6.82, 2.895, new Rotation2d(calcAngle(-Math.PI/2, isRed))),
            List.of(new Translation2d(b * -6.4, 3.395)),
            ampPose,
            config);

        // Amp -> amp note
        Trajectory driveToAmpNote = TrajectoryGenerator.generateTrajectory(
            ampPose,
            List.of(new Translation2d(b * -6.27, 3.395)),
            ampNotePose,
            config);

        // Amp note -> amp
        Trajectory driveToAmpFromNote = TrajectoryGenerator.generateTrajectory(
            ampNotePose,
            List.of(new Translation2d(b * -6.27, 3.395)),
            ampPose,
            config);

        // Amp -> end of wing
        Trajectory driveAcrossLineAmp = TrajectoryGenerator.generateTrajectory(
            ampPose,
            List.of(new Translation2d(b * -6.07, 3.295)),
            new Pose2d(b * -1.27, 2.395, new Rotation2d(calcAngle(0, isRed))),
            config);

        // Side of speaker -> source side
        Trajectory driveForwardAndOut = TrajectoryGenerator.generateTrajectory(
            sideSpeakerPose,
            List.of(new Translation2d(b * -6.5, -1.6)),
            new Pose2d(b * -5, -2.8, new Rotation2d(calcAngle(0, isRed))),
            config);

        // Speaker -> amp note
        Trajectory driveSpeakerToAmpNote = TrajectoryGenerator.generateTrajectory(
            speakerPose,
            List.of(new Translation2d(b * -5.7, 1.8)),
            new Pose2d(b * -5.57, 2.945, new Rotation2d(calcAngle(Math.PI/3, isRed))),
            config);

        // Amp note -> speaker
        Trajectory driveAmpNoteToSpeaker = TrajectoryGenerator.generateTrajectory(
            ampNotePose,
            List.of(),
            speakerPose,
            config);

        // Speaker -> source note
        Trajectory driveSpeakerToSourceNote = TrajectoryGenerator.generateTrajectory(
            speakerPose,
            List.of(),
            sourceNotePose,
            config);

        // Source note -> speaker
        Trajectory driveSourceNoteToSpeaker = TrajectoryGenerator.generateTrajectory(
            sourceNotePose,
            List.of(),
            speakerPose,
            config);

        middleDoubleScore.add(driveForward);
        middleDoubleScore.add(driveBackward);
        middleDoubleScore.add(driveFartherForward);

        tripleSpeakerScore.add(driveSpeakerToAmpNote);
        tripleSpeakerScore.add(driveAmpNoteToSpeaker);
        tripleSpeakerScore.add(driveSpeakerToSourceNote);
        tripleSpeakerScore.add(driveSourceNoteToSpeaker);

        ampDoubleScore.add(driveToAmp);
        ampDoubleScore.add(driveToAmpNote);
        ampDoubleScore.add(driveToAmpFromNote);
        ampDoubleScore.add(driveAcrossLineAmp);

        driveForwardAuto.add(driveForwardAndOut);

        tripleSpeakerScore.add(driveSpeakerToAmpNote);
    }

    private double calcAngle(double angle, Boolean isRed) {
        if (isRed) {
            if (angle > 0) angle -= Math.PI;
            else angle += Math.PI;
            angle *= -1;
        }

        return angle;
    }

    // Half x = 8.27
    // Half y = 4.105
    // Speaker Pose2d(-6.82, 1.295, 0 or Math.PI)   Speaker Pose2d(1.45, 5.4, 0 or Math.PI)
    // 

    public Trajectory PathFromCurrentPose(Pose2d targetPose) {
        
        Translation2d inBetween = new Translation2d(robotDrive.getPose().getX() + targetPose.getX() / 2, robotDrive.getPose().getY() + targetPose.getY() / 2);
        
        return TrajectoryGenerator.generateTrajectory(
            robotDrive.getPose(),
            List.of(inBetween),
            targetPose,
            config);
    }

    public Trajectory MultiplePathFromCurrentPose(Translation2d[] listOfPoses, Pose2d targetPose) {
        return TrajectoryGenerator.generateTrajectory(
            robotDrive.getPose(),
            List.of(listOfPoses),
            targetPose,
            config);
    }
}