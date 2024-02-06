// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class RunArmCommand extends Command {
  /** Creates a new RunIntakeCommand. */

  ArmSubsystem m_ArmSubsystem;
  double m_speed;

  public RunArmCommand(double speed, ArmSubsystem ArmSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ArmSubsystem = ArmSubsystem;
    m_speed = speed;

    addRequirements(ArmSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_ArmSubsystem.RunArm(m_speed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_ArmSubsystem.RunArm(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
