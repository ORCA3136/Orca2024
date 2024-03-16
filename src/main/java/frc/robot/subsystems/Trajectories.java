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

    private ArrayList<Trajectory> middleDoubleScoreBlue = new ArrayList<Trajectory>();
    private ArrayList<Trajectory> tripleSpeakerScoreBlue = new ArrayList<Trajectory>();
    private ArrayList<Trajectory> ampDoubleScoreBlue = new ArrayList<Trajectory>();
    private ArrayList<Trajectory> driveForwardAutoBlue = new ArrayList<Trajectory>();

    private ArrayList<Trajectory> middleDoubleScoreRed = new ArrayList<Trajectory>();
    private ArrayList<Trajectory> tripleSpeakerScoreRed = new ArrayList<Trajectory>();
    private ArrayList<Trajectory> ampDoubleScoreRed = new ArrayList<Trajectory>();
    private ArrayList<Trajectory> driveForwardAutoRed = new ArrayList<Trajectory>();

    TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(false);

    public Trajectories(DriveSubsystem drive) {
        robotDrive = drive;

        CreateBlueTrajectories();
        CreateRedTrajectories();
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

    public ArrayList<Trajectory> getMiddleDoubleScore(boolean red) {
        if (red) return middleDoubleScoreRed;
        return middleDoubleScoreBlue;
    }

    public ArrayList<Trajectory> getTripleSpeakerScore(boolean red) {
        if (red) return tripleSpeakerScoreRed;
        return tripleSpeakerScoreBlue;
    }

    public ArrayList<Trajectory> getAmpDoubleScore(boolean red) {
        if (red) return ampDoubleScoreRed;
        return ampDoubleScoreBlue;
    }

    public ArrayList<Trajectory> getDriveForward(boolean red) {
        if (red) return driveForwardAutoRed;
        return driveForwardAutoBlue;
    }

    public void CreateRedTrajectories() {
        
        double b = -1;

        Pose2d speakerPose = new Pose2d(b * -6.8, 1.295, new Rotation2d(calcAngle(0, true)));
        Pose2d sideSpeakerPose = new Pose2d(b * -7.42, 0.13, new Rotation2d(calcAngle(0, true)));
        Pose2d ampPose = new Pose2d(b * -6.5, 3.75, new Rotation2d(calcAngle(-Math.PI/2, true)));
        Pose2d ampNotePose = new Pose2d(b * -5.57, 3.2, new Rotation2d(calcAngle(0, true)));
        Pose2d middleNotePose = new Pose2d(b * -5.67, 1.295, new Rotation2d(calcAngle(0, true)));
        Pose2d sourceNotePose = new Pose2d(b * -5.95, 0, new Rotation2d(calcAngle(0, true)));
        
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
            new Pose2d(b * -4.67, 1.295, new Rotation2d(calcAngle(0, true))),
            config);

        // Corner of starting zone -> Amp
        Trajectory driveToAmp = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -6.82, 2.895, new Rotation2d(calcAngle(-Math.PI/2, true))),
            List.of(new Translation2d(b * -6.4, 3.395)),
            ampPose,
            config);

        // Amp -> amp note
        Trajectory driveToAmpNote = TrajectoryGenerator.generateTrajectory(
            ampPose,
            List.of(new Translation2d(b * -6.27, 3.4)),
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
            new Pose2d(b * -1.27, 2.395, new Rotation2d(calcAngle(0, true))),
            config);

        // Side of speaker -> source side
        Trajectory driveForwardAndOut = TrajectoryGenerator.generateTrajectory(
            sideSpeakerPose,
            List.of(new Translation2d(b * -6.5, -1.6)),
            new Pose2d(b * -2, -2.8, new Rotation2d(calcAngle(0, true))),
            config);

        // Speaker -> amp note
        Trajectory driveSpeakerToAmpNote = TrajectoryGenerator.generateTrajectory(
            speakerPose,
            List.of(new Translation2d(b * -5.7, 1.8)),
            new Pose2d(b * -5.57, 2.945, new Rotation2d(calcAngle(Math.PI/2, true))),
            config);

        // Amp note -> speaker
        Trajectory driveAmpNoteToSpeaker = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -5.57, 2.945, new Rotation2d(calcAngle(Math.PI/3, true))),
            List.of(new Translation2d(b * -5.7, 1.8)),
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

        middleDoubleScoreRed.add(driveForward);
        middleDoubleScoreRed.add(driveBackward);
        middleDoubleScoreRed.add(driveFartherForward);

        tripleSpeakerScoreRed.add(driveSpeakerToAmpNote);
        tripleSpeakerScoreRed.add(driveAmpNoteToSpeaker);
        tripleSpeakerScoreRed.add(driveSpeakerToSourceNote);
        tripleSpeakerScoreRed.add(driveSourceNoteToSpeaker);

        ampDoubleScoreRed.add(driveToAmp);
        ampDoubleScoreRed.add(driveToAmpNote);
        ampDoubleScoreRed.add(driveToAmpFromNote);
        ampDoubleScoreRed.add(driveAcrossLineAmp);

        driveForwardAutoRed.add(driveForwardAndOut);
    }

    public void CreateBlueTrajectories() {
        
        double b = 1;

        Pose2d speakerPose = new Pose2d(b * -6.8, 1.295, new Rotation2d(calcAngle(0, false)));
        Pose2d sideSpeakerPose = new Pose2d(b * -7.42, 0.13, new Rotation2d(calcAngle(0, false)));
        Pose2d ampPose = new Pose2d(b * -6.52, 3.75, new Rotation2d(calcAngle(-Math.PI/2, false)));
        Pose2d ampNotePose = new Pose2d(b * -5.57, 3.2, new Rotation2d(calcAngle(0.2, false)));
        Pose2d middleNotePose = new Pose2d(b * -5.67, 1.295, new Rotation2d(calcAngle(0, false)));
        Pose2d sourceNotePose = new Pose2d(b * -5.95, 0, new Rotation2d(calcAngle(0, false)));
        
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
            new Pose2d(b * -4.67, 1.295, new Rotation2d(calcAngle(0, false))),
            config);

        // Corner of starting zone -> Amp
        Trajectory driveToAmp = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -6.82, 2.895, new Rotation2d(calcAngle(-Math.PI/2, false))),
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
            new Pose2d(b * -1.27, 2.395, new Rotation2d(calcAngle(0, false))),
            config);

        // Side of speaker -> source side
        Trajectory driveForwardAndOut = TrajectoryGenerator.generateTrajectory(
            sideSpeakerPose,
            List.of(new Translation2d(b * -6.5, -1.6)),
            new Pose2d(b * -2, -2.8, new Rotation2d(calcAngle(0, false))),
            config);

        // Speaker -> amp note
        Trajectory driveSpeakerToAmpNote = TrajectoryGenerator.generateTrajectory(
            speakerPose,
            List.of(new Translation2d(b * -5.7, 1.8)),
            new Pose2d(b * -5.57, 2.945, new Rotation2d(calcAngle(Math.PI/2, false))),
            config);

        // Amp note -> speaker
        Trajectory driveAmpNoteToSpeaker = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -5.57, 2.945, new Rotation2d(calcAngle(Math.PI/3, false))),
            List.of(new Translation2d(b * -5.7, 1.8)),
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

        middleDoubleScoreBlue.add(driveForward);
        middleDoubleScoreBlue.add(driveBackward);
        middleDoubleScoreBlue.add(driveFartherForward);

        tripleSpeakerScoreBlue.add(driveSpeakerToAmpNote);
        tripleSpeakerScoreBlue.add(driveAmpNoteToSpeaker);
        tripleSpeakerScoreBlue.add(driveSpeakerToSourceNote);
        tripleSpeakerScoreBlue.add(driveSourceNoteToSpeaker);

        ampDoubleScoreBlue.add(driveToAmp);
        ampDoubleScoreBlue.add(driveToAmpNote);
        ampDoubleScoreBlue.add(driveToAmpFromNote);
        ampDoubleScoreBlue.add(driveAcrossLineAmp);

        driveForwardAutoBlue.add(driveForwardAndOut);
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