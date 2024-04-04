package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;


public class AutoSpeakerCentering extends Command {

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
  boolean finished = false;

  public AutoSpeakerCentering(ShooterSubsystem ShooterSubsystem, SensorSubsystem SensorSubsystem, 
  ArmSubsystem ArmSubsystem, IntakeSubsystem IntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_SensorSubsystem = SensorSubsystem;
    m_ShooterSubsystem = ShooterSubsystem;
    m_ArmSubsystem = ArmSubsystem;
    m_IntakeSubsystem = IntakeSubsystem;

    addRequirements(ShooterSubsystem, IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startedShot = false;
    finished = false;
    DataLogManager.log("Auto shooting - init");

    // m_DriveSubsystem.speakerCentering(m_controller, m_SensorSubsystem).schedule();
    m_ShooterSubsystem.setShootSpeed(m_SensorSubsystem.speedMap);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // DataLogManager.log("Auto shooting --- execute");

    NetworkTableInstance.getDefault().getTable("AutoCentering").getEntry("ShooterAtSpeed").setBoolean(m_ShooterSubsystem.getSpeed() > m_SensorSubsystem.speedMap - 300);
    NetworkTableInstance.getDefault().getTable("AutoCentering").getEntry("ArmInRange").setBoolean(m_ArmSubsystem.getError() > -2.5 && m_ArmSubsystem.getError() < 0);
    NetworkTableInstance.getDefault().getTable("AutoCentering").getEntry("RotatonInRange").setBoolean(m_SensorSubsystem.getCenteringRotationError() < 2);

    m_ArmSubsystem.setTrapezoidalSetpoint(m_SensorSubsystem.angleMap);
    m_ArmSubsystem.AutomaticPositioning();
    m_ShooterSubsystem.updateSetpointOnly(m_SensorSubsystem.speedMap + m_SensorSubsystem.verticalOffset);

    if (!startedShot && m_ShooterSubsystem.getSpeed() > m_SensorSubsystem.speedMap - 300 && 
        m_ArmSubsystem.getError() > -2.5 && m_ArmSubsystem.getError() < 0 && m_SensorSubsystem.getCenteringRotationError() < 2) {
      // DataLogManager.log("Auto shooting ----------- started shot");
      startedShot = true;
      m_IntakeSubsystem.RunIntake(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // m_DriveSubsystem.regularDrive(m_controller).schedule();
    m_ShooterSubsystem.setShootSpeed(0);
    m_IntakeSubsystem.RunIntake(0);

    // DataLogManager.log("Auto shooting --------- end");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return startedShot && !m_SensorSubsystem.getIntakeSensor(2) && !m_SensorSubsystem.getIntakeSensor(1);
  }
}
