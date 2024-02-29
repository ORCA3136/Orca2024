package frc.robot.subsystems;

import java.util.Optional;

import com.choreo.lib.Choreo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ModuleConstants;

public class TrajectoryGenerator {
    
    private PIDController AutoDrivePID = new PIDController(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD);
    private PIDController AutoTurnPID = new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD);

    private DriveSubsystem robotDrive;

    public TrajectoryGenerator(DriveSubsystem drive) {
        robotDrive = drive;
    }

    public Command GenerateTrajectory() {
        return null;
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
}
