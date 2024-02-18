package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunShooterCommand;

public class NoteOffIntake extends Command {
    /** Creates a new RunIntakeCommand. */

  IntakeSubsystem m_IntakeSubsystem;
  ShooterSubsystem m_ShooterSubsystem;
  SensorSubsystem m_SensorSubsystem;
  boolean m_IntakeSensorValue = false;
  boolean finished = false;
  boolean finishedIntake = false;

  Command intakeCommand;
  Command parallelCommand;

  public NoteOffIntake(ShooterSubsystem ShooterSubsystem, IntakeSubsystem IntakeSubsystem, SensorSubsystem SensorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = IntakeSubsystem;
    m_ShooterSubsystem = ShooterSubsystem;
    m_SensorSubsystem = SensorSubsystem;

    intakeCommand = new RunIntakeCommand(0.5, m_IntakeSubsystem).withTimeout(0.5);
    parallelCommand = new ParallelCommandGroup(new RunIntakeCommand(-0.05, m_IntakeSubsystem), new RunShooterCommand(-0.05, m_ShooterSubsystem)).withTimeout(0.5);

    addRequirements(IntakeSubsystem, ShooterSubsystem, SensorSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_IntakeSensorValue = m_SensorSubsystem.getIntakeSensor(0);
    if (m_IntakeSensorValue == true) finished = true;

    intakeCommand = new RunIntakeCommand(0.5, m_IntakeSubsystem).withTimeout(0.5);
    intakeCommand.schedule();

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
    if (!intakeCommand.isFinished() && !finishedIntake) finishedIntake = true;

    if (finishedIntake) {

        if (!parallelCommand.isScheduled()) parallelCommand.schedule();

        if (parallelCommand.isFinished()) {
            end(false);
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (intakeCommand.isScheduled()) intakeCommand.cancel();
    if (parallelCommand.isScheduled()) parallelCommand.cancel();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IntakeSensorValue;
  }
}
