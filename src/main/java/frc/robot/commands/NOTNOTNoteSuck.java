package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.LimelightHelpers;

public class NOTNOTNoteSuck extends Command {
  /** Creates a new AprilTagFollowCommand. */

  DriveSubsystem m_robotDrive;
  String limelightName = "limelight-note";

  public NOTNOTNoteSuck(DriveSubsystem robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_robotDrive = robotDrive;

    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Positive: Target is right of cursor
    double m_tx = LimelightHelpers.getTX(limelightName);

    double xSpeed = 0.4;

    double rot = 0;
    if (m_tx < -8) {
      rot = 0.15;
    } else if (m_tx < -2) {
        rot = 0.1;
    } else if (m_tx > 8) {
        rot = -0.15;
    } else if (m_tx > 2) {
      rot = -0.1;
    }
    

    m_robotDrive.drive(xSpeed, 0, rot, false, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
