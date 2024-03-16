package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.LimelightHelpers;

public class NOTNOTNoteSuck extends Command {
  /** Creates a new AprilTagFollowCommand. */

  DriveSubsystem m_robotDrive;
  IntakeSubsystem m_IntakeSubsystem;
  SensorSubsystem m_SensorSubsystem;
  ShooterSubsystem m_ShooterSubsystem;

  String limelightName = "limelight-note";

  public NOTNOTNoteSuck(DriveSubsystem robotDrive, IntakeSubsystem intake, SensorSubsystem sensor, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_robotDrive = robotDrive;
    m_IntakeSubsystem = intake;
    m_SensorSubsystem = sensor;
    m_ShooterSubsystem = shooter;

    addRequirements(robotDrive, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // DataLogManager.log("NOTNOTNoteSuck Init");
    m_IntakeSubsystem.RunIntake(0.5);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // DataLogManager.log("NOTNOTNoteSuck Execute");

    // if (m_SensorSubsystem.getIntakeSensor(1)) {
    //   end(false);
    // }

    //Positive: Target is right of cursor
    double m_tx = LimelightHelpers.getTX(limelightName);

    double xSpeed = 0.3;

    double rot = 0;
    if (m_tx < -8) {
      rot = 0.15;
    } else if (m_tx < -3) {
        rot = 0.1;
    } else if (m_tx > 8) {
        rot = -0.15;
    } else if (m_tx > 3) {
      rot = -0.1;
    }
    m_robotDrive.drive(xSpeed, 0, rot, false, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // DataLogManager.log("NOTNOTNoteSuck End -------");
    m_IntakeSubsystem.RunIntake(0);
    if (DriverStation.isTeleop())
      new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(1.5).schedule();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_SensorSubsystem.getIntakeSensor(1);
  }
}
