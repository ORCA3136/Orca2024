package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.LimelightHelpers;
//import com.pathplanner.lib.*;
import com.pathplanner.lib.commands.PathfindHolonomic;

public class AutoGotoCommunity extends Command {
    
    DriveSubsystem m_robotDrive;
    String limelightName = "limelight";

    public AutoGotoCommunity(DriveSubsystem robotDrive) {
        m_robotDrive = robotDrive;
    
        addRequirements(robotDrive);
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //PathfindHolonomic 
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
