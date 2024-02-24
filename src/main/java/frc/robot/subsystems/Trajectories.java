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
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class Trajectories {
    
    private PIDController AutoDrivePID = new PIDController(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD);
    private PIDController AutoTurnPID = new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD);

    private DriveSubsystem robotDrive;

    private ArrayList<Trajectory> middleDoubleScore = new ArrayList<Trajectory>();
    private ArrayList<Trajectory> ampDoubleScore = new ArrayList<Trajectory>();
    private ArrayList<Trajectory> driveForwardAuto = new ArrayList<Trajectory>();

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
        j = ampDoubleScore.size();
        for (int i = 0; i < j; i ++) {
            ampDoubleScore.remove(0);
        }
        j = driveForwardAuto.size();
        for (int i = 0; i < j; i ++) {
            driveForwardAuto.remove(0);
        }

        boolean isRed = red;

        double a, b, c;
        a = 0;
        b = 1;
        c = 0;

        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                b = -1;
                c = Math.PI;
            }
            else {
                b = 1;
                c = 0;
            }
        } else {
            if (isRed) {
                b = -1;
                c = Math.PI;
            }
            else {
                b = 1;
                c = 0;
            }
        }

        // Configure trajectory stuff
        // For forwards
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(false);


        
        // Trajectories
        Trajectory driveForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -6.82, 1.295, new Rotation2d(c)),
            List.of(new Translation2d(b * -6.27, 1.395)),
            new Pose2d(b * -5.67, 1.295, new Rotation2d(c)),
            config);

        Trajectory driveBackward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -5.67, 1.295, new Rotation2d(c)),
            List.of(new Translation2d(b * -6.27, 1.195)),
            new Pose2d(b * -6.82, 1.295, new Rotation2d(c)),
            config);

        Trajectory driveFartherForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -6.82, 1.295, new Rotation2d(c)),
            List.of(new Translation2d(b * -5.87, 1.395)),
            new Pose2d(b * -4.67, 1.295, new Rotation2d(c)),
            config);

        Trajectory driveToAmp = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -6.82, 2.895, new Rotation2d(-Math.PI/2)),
            List.of(new Translation2d(b * -6.4, 3.395)),
            new Pose2d(b * -6.52, 3.65, new Rotation2d(-Math.PI/2)),
            config);

        Trajectory driveToAmpNote = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -6.52, 3.65, new Rotation2d(-Math.PI/2)),
            List.of(new Translation2d(b * -6.27, 3.395)),
            new Pose2d(b * -5.57, 2.945, new Rotation2d(c)),
            config);

        Trajectory driveToAmpFromNote = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -5.77, 3.145, new Rotation2d(c)),
            List.of(new Translation2d(b * -6.27, 3.395)),
            new Pose2d(b * -6.52, 3.7, new Rotation2d(-Math.PI/2)),
            config);

        Trajectory driveAcrossLineAmp = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -6.52, 3.65, new Rotation2d(-Math.PI/2)),
            List.of(new Translation2d(b * -6.07, 3.295)),
            new Pose2d(b * -1.27, 2.395, new Rotation2d(c)),
            config);

        Trajectory driveForwardAndOut = TrajectoryGenerator.generateTrajectory(
            new Pose2d(b * -7.42, 0.13, new Rotation2d(c)),
            List.of(new Translation2d(b * -6.5, -1.6)),
            new Pose2d(b * -5, -2.8, new Rotation2d(-Math.PI/2)),
            config);

        middleDoubleScore.add(driveForward);
        middleDoubleScore.add(driveBackward);
        middleDoubleScore.add(driveFartherForward);

        ampDoubleScore.add(driveToAmp);
        ampDoubleScore.add(driveToAmpNote);
        ampDoubleScore.add(driveToAmpFromNote);
        ampDoubleScore.add(driveAcrossLineAmp);

        driveForwardAuto.add(driveForwardAndOut);
    }
}