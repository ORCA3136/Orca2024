package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;

public class AprilTagFollowCommand extends Command {
  /** Creates a new AprilTagFollowCommand. */

  String limelightName = "limelight";

  public AprilTagFollowCommand() {
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Positive: Target is right of cursor
    double m_tx = LimelightHelpers.getTX(limelightName);
    //Positive: Target is above cursor
    double m_ty = LimelightHelpers.getTY(limelightName);
    //Bigger number means closer
    double m_ta = LimelightHelpers.getTA(limelightName);

    double xSpeed = 0.0;
    if (m_ta > 0.1 && m_ta < 4) {
      xSpeed = 0.2;
    }

    double rot = 0;
    if (m_tx < -3) {
      rot = 0.03;
    } else if (m_tx > 3) {
      rot = -0.03;
    }


    
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
