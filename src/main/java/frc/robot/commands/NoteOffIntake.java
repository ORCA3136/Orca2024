package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class NoteOffIntake extends Command {
    /** Creates a new RunIntakeCommand. */

  IntakeSubsystem m_IntakeSubsystem;
  ShooterSubsystem m_ShooterSubsystem;
  SensorSubsystem m_SensorSubsystem;
  boolean m_IntakeSensorValue = true;
  boolean finished = false;
  boolean finishedIntake = false;

  Command intakeCommand;
  Command parallelCommand;

  public NoteOffIntake(ShooterSubsystem ShooterSubsystem, IntakeSubsystem IntakeSubsystem, SensorSubsystem SensorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = IntakeSubsystem;
    m_ShooterSubsystem = ShooterSubsystem;
    m_SensorSubsystem = SensorSubsystem;
    addRequirements(IntakeSubsystem, ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_IntakeSensorValue = m_SensorSubsystem.getIntakeSensor(0);
    if (m_IntakeSensorValue == true) finished = true;
    m_ShooterSubsystem.setShootSpeed(-600);
    m_IntakeSubsystem.RunIntake(-0.1);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

/**
 * Intake in with timeout
 * 
 * vvv
 * 
 * Shooter + Intake out slow
 * until
 * Sensor detects note
 * 
 * vvv
 * 
 * End command
 */

    m_IntakeSensorValue = m_SensorSubsystem.getIntakeSensor(0);

    if (!m_IntakeSensorValue) {
      m_IntakeSubsystem.RunIntake(0);
      m_ShooterSubsystem.setShootSpeed(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_IntakeSubsystem.RunIntake(0);
    m_ShooterSubsystem.setShootSpeed(0);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_IntakeSensorValue;
  }
}
