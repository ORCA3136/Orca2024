package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootSpeaker extends Command {
    /** Creates a new RunIntakeCommand. */

  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;

  Command intakeCommand = new RunIntakeCommand(-0.05, m_IntakeSubsystem).withTimeout(1);

  boolean startedIntake = false;

  public ShootSpeaker(ShooterSubsystem ShooterSubsystem, IntakeSubsystem IntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = IntakeSubsystem;
    m_ShooterSubsystem = ShooterSubsystem;

    addRequirements(IntakeSubsystem, ShooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_ShooterSubsystem.setNewTarget(Constants.ShooterConstants.shoot);

  }

// ParallelCommandGroup shootNote = new ParallelCommandGroup(RunShooterCommand(1), new SequentialCommandGroup(new WaitCommand(3), RunIntakeCommand(0.5)));

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  /**
   * Start flywheel
   * until
   * Max/consistent speed
   * 
   * vvv
   * 
   * Keep flywheel at max speed and runIntake 
   * with timeout
   */

    if (!startedIntake && m_ShooterSubsystem.atSpeed()) {

        startedIntake = true;
        intakeCommand.schedule();

    }
    if (startedIntake && intakeCommand.isFinished()) 
        end(false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (intakeCommand.isScheduled()) intakeCommand.cancel();
    m_ShooterSubsystem.setNewTarget(Constants.ShooterConstants.stop);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
