package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class NoteOffFlywheel extends Command {
    /** Creates a new RunIntakeCommand. */

  IntakeSubsystem m_IntakeSubsystem;
  ShooterSubsystem m_ShooterSubsystem;
  SensorSubsystem m_SensorSubsystem;

  boolean m_IntakeSensorValue;
  boolean m_IntakeSideTopSensorValue;
  boolean m_IntakeSideBottomSensorValue;
  boolean finished = false;
  boolean finishedIntake = false;

  boolean onTopSensor = false;

  public NoteOffFlywheel(ShooterSubsystem ShooterSubsystem, IntakeSubsystem IntakeSubsystem, SensorSubsystem SensorSubsystem) {
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
    m_IntakeSideBottomSensorValue = m_SensorSubsystem.getIntakeSensor(1);
    m_IntakeSideTopSensorValue = m_SensorSubsystem.getIntakeSensor(2);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_IntakeSensorValue = m_SensorSubsystem.getIntakeSensor(0);
    m_IntakeSideBottomSensorValue = m_SensorSubsystem.getIntakeSensor(1);
    m_IntakeSideTopSensorValue = m_SensorSubsystem.getIntakeSensor(2);



    // Both side sensors are engaged - end
    if (m_IntakeSideTopSensorValue && m_IntakeSideBottomSensorValue) end(false);
    // No note in intake
    else if (!m_IntakeSideTopSensorValue && !m_IntakeSideBottomSensorValue) end(false);
    // Bottom sensor is engaged - intake in
    else if (m_IntakeSideBottomSensorValue) {
      m_IntakeSubsystem.RunIntake(0.15);
      if (onTopSensor) {
        m_ShooterSubsystem.setNewTarget(0);
        onTopSensor = false;
      }
    }
    // Top sensor is engaged - intake out
    else if (m_IntakeSideTopSensorValue) {
      m_IntakeSubsystem.RunIntake(-0.1);
      if (!onTopSensor) {
        m_ShooterSubsystem.setNewTarget(-500);
        onTopSensor = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_IntakeSubsystem.RunIntake(0);
    m_ShooterSubsystem.setNewTarget(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}