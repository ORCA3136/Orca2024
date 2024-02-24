package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootSpeaker extends Command {
    /** Creates a new RunIntakeCommand. */

  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;

  Command intakeCommand;

  boolean startedIntake = false;
  double shooterSpeed;

  public ShootSpeaker(ShooterSubsystem ShooterSubsystem, IntakeSubsystem IntakeSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = IntakeSubsystem;
    m_ShooterSubsystem = ShooterSubsystem;
    shooterSpeed = speed;

    addRequirements(IntakeSubsystem, ShooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_ShooterSubsystem.setShootSpeed(shooterSpeed);
    startedIntake = false;

    new WaitCommand(2).andThen(() -> {
      if (!startedIntake) {
        m_IntakeSubsystem.RunIntake(1);
        startedIntake = true;
        new WaitCommand(0.5).andThen(() -> {end(false);});
      }
    });
  }

// ParallelCommandGroup shootNote = new ParallelCommandGroup(RunShooterCommand(1), new SequentialCommandGroup(new WaitCommand(3), RunIntakeCommand(0.5)));

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!startedIntake && m_ShooterSubsystem.getSpeed() > shooterSpeed) {
      m_IntakeSubsystem.RunIntake(1);
      new WaitCommand(0.5).andThen(() -> {end(false);}); 
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
    return false;
  }
}
