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

/* TO DO
 * 
 * Continue learning pathplanning
 *    Auto paths
 *    Teleop paths
 * Update limelight pose -- Improve
 * 
 * Improve FRC web components dashboard
 * Data logs 
 * Rio configurable brownout levels
 * 
 * 
 *   NoteOffIntake ----- Intake routine
 * Sensor calls subroutine (prevent flickers) -> intake all the way in -> shooter and intake slow out until sensor something
 * 
 *   ShootSpeaker ----- Shooter routine
 * Ramp up until max speed or consistent -> intake in
 * 
 * Centering routine
 * (Maybe use sensor to run when near speaker and we have a note) Use apriltags to determine distance to speaker -> do math to determine angle and rotation
 * 
 * Arm up and down
 * PID without buttons -- Have a resting level -- Note and button for amp -- Note and button/near speaker
 * Down when near stage -- Have a level for player station -- All the way down for intake
 * 
 * 
 * Limit switch
 * YAGSL
 * 2nd limelight
 * Beambreak sensor
 * 
 * Gyro value vs Locked value
 */


/* Done
 * 
 * Arm encoder
 * PID arm positions
 */