package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
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
  XboxController m_controller;

  // Angle to speaker
  // Distance to speaker
  // Current speed for offset

  double shooterTarget;
  double armTarget;

  double shooterSpeed;
  double armPosition;

  boolean startedShot = false;

  public SpeakerCentering(ShooterSubsystem ShooterSubsystem, SensorSubsystem SensorSubsystem, ArmSubsystem ArmSubsystem,
        DriveSubsystem DriveSubsystem, IntakeSubsystem IntakeSubsystem, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_SensorSubsystem = SensorSubsystem;
    m_ShooterSubsystem = ShooterSubsystem;
    m_ArmSubsystem = ArmSubsystem;
    m_IntakeSubsystem = IntakeSubsystem;
    m_DriveSubsystem = DriveSubsystem;
    m_controller = controller;

    addRequirements(ShooterSubsystem, ArmSubsystem, IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startedShot = false;
    // DataLogManager.log("Auto shooting - init");

    m_DriveSubsystem.speakerCentering(m_controller, m_SensorSubsystem).schedule();
    m_ShooterSubsystem.setShootSpeed(5500);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // DataLogManager.log("Auto shooting --- execute");

    NetworkTableInstance.getDefault().getTable("AutoCentering").getEntry("Stopped").setBoolean(m_DriveSubsystem.stopped());
    NetworkTableInstance.getDefault().getTable("AutoCentering").getEntry("InRange").setBoolean(m_SensorSubsystem.inRange());
    NetworkTableInstance.getDefault().getTable("AutoCentering").getEntry("ShooterAtSpeed").setBoolean(m_ShooterSubsystem.getSpeed() > 4000);
    NetworkTableInstance.getDefault().getTable("AutoCentering").getEntry("StartedShot").setBoolean(startedShot);
    NetworkTableInstance.getDefault().getTable("AutoCentering").getEntry("ArmInRange").setBoolean(m_ArmSubsystem.getError() < 1);

    m_ArmSubsystem.SetSensorPID(m_SensorSubsystem);

    if (m_DriveSubsystem.stopped() && m_SensorSubsystem.inRange()) {
      // DataLogManager.log("Auto shooting ----- In range & stopped");
      if (!startedShot && m_ShooterSubsystem.getSpeed() > 3800 && m_ArmSubsystem.getError() < 0.5) {
        // DataLogManager.log("Auto shooting ------------- Started shot --------");
        startedShot = true;
        m_IntakeSubsystem.RunIntake(1);
        // Wait 0.5 seconds
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // DataLogManager.log("Auto shooting --- End --------------------------------------------------");
    m_ShooterSubsystem.setShootSpeed(0);
    m_ArmSubsystem.SetPositionPID(Constants.ArmPIDConstants.STAGE);
    m_IntakeSubsystem.RunIntake(0);
    m_DriveSubsystem.regularDrive(m_controller).schedule();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
