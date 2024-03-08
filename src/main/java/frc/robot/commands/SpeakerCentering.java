package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.Constants;


public class SpeakerCentering extends Command {

  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  ArmSubsystem m_ArmSubsystem;
  DriveSubsystem m_DriveSubsystem;
  SensorSubsystem m_SensorSubsystem;

  // Angle to speaker
  // Distance to speaker
  // Current speed for offset

  double shooterTarget;
  double armTarget;

  double shooterSpeed;
  double armPosition;

  public SpeakerCentering(ShooterSubsystem ShooterSubsystem, SensorSubsystem SensorSubsystem, ArmSubsystem ArmSubsystem,
        DriveSubsystem DriveSubsystem, IntakeSubsystem IntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_SensorSubsystem = SensorSubsystem;
    m_ShooterSubsystem = ShooterSubsystem;
    m_ArmSubsystem = ArmSubsystem;
    m_IntakeSubsystem = IntakeSubsystem;
    m_DriveSubsystem = DriveSubsystem;

    addRequirements(ShooterSubsystem, ArmSubsystem, IntakeSubsystem, DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

// ParallelCommandGroup shootNote = new ParallelCommandGroup(RunShooterCommand(1), new SequentialCommandGroup(new WaitCommand(3), RunIntakeCommand(0.5)));

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {



    m_ShooterSubsystem.shootNoteNOTNOTSensor(m_SensorSubsystem);
    m_ArmSubsystem.SetPIDNOTNOTSensor(m_SensorSubsystem);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_ShooterSubsystem.setShootSpeed(0);
    m_ArmSubsystem.SetPIDPosition(Constants.ArmPIDConstants.STAGE);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
