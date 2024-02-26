package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootSpeaker extends Command {
    /** Creates a new RunIntakeCommand. */

  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;

  boolean finished = false;
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

  }

// ParallelCommandGroup shootNote = new ParallelCommandGroup(RunShooterCommand(1), new SequentialCommandGroup(new WaitCommand(3), RunIntakeCommand(0.5)));

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    NetworkTableInstance.getDefault().getTable("ShootSpeaker").getEntry("Finished").setBoolean(finished);

    if (finished) end(false);

    if (!startedIntake && m_ShooterSubsystem.getSpeed() > shooterSpeed - 300) {
      m_IntakeSubsystem.RunIntake(1);
      startedIntake = true;
      finished = true;
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
    return finished;
  }
}
