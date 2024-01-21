// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterCommand extends Command {
  /** Creates a new RunIntakeCommand. */

  ShooterSubsystem m_ShooterSubsystem;
  double m_speed;

  public RunShooterCommand(double speed, ShooterSubsystem ShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSubsystem = ShooterSubsystem;
    m_speed = speed;

    addRequirements(ShooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_ShooterSubsystem.RunShooter(m_speed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_ShooterSubsystem.RunShooter(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
