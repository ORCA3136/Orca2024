package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.Constants;


public class TestSpeakerCentering extends Command {

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

  public TestSpeakerCentering(SensorSubsystem SensorSubsystem, ArmSubsystem ArmSubsystem,
        DriveSubsystem DriveSubsystem, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_SensorSubsystem = SensorSubsystem;
    m_ArmSubsystem = ArmSubsystem;
    m_DriveSubsystem = DriveSubsystem;
    m_controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startedShot = false;
    finished = false;

    m_DriveSubsystem.speakerCentering(m_controller, m_SensorSubsystem).schedule();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_ArmSubsystem.setTrapezoidalSetpoint(m_SensorSubsystem.angleMap);

    if (!startedShot && m_ArmSubsystem.getError() < 0.5 && m_SensorSubsystem.getCenteringRotationError() < 15) {
      // DataLogManager.log("Auto shooting ------------- Started shot --------");
      startedShot = true;
      new SequentialCommandGroup(Commands.waitSeconds(0.5), Commands.runOnce(() -> {this.finished = true;}));
    }
   
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_ArmSubsystem.setTrapezoidalSetpoint(25);
    m_DriveSubsystem.regularDrive(m_controller).schedule();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
