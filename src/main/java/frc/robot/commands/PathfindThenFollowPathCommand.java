// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class PathfindThenFollowPathCommand extends Command {
  /** Creates a new RunIntakeCommand. */

  Command pathCommand;
  DriveSubsystem m_DriveSubsystem;
  String path;

  public PathfindThenFollowPathCommand(DriveSubsystem driveSubsystem, String pathName) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = driveSubsystem;
    path = pathName;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  DataLogManager.log(">>PathfindThenFollowPathCommant:Initalize");

    pathCommand = m_DriveSubsystem.pathfindThenFollowPathCommand(path);

    // schedule the autonomous command (example)
    pathCommand.schedule();
  DataLogManager.log("<<PathfindThenFollowPathCommant:Initalize");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    pathCommand.cancel();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
