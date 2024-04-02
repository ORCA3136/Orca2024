package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TurnToAngle extends Command {
    
    DriveSubsystem m_robotDrive;
    SensorSubsystem m_SensorSubsystem;

    double rot;
    double rotationDifference;

    double currentAngle;
    double angleToRotate;
    double targetAngle;

    public TurnToAngle(DriveSubsystem robotDrive, SensorSubsystem sensor, double angleToRotate) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_robotDrive = robotDrive;
    m_SensorSubsystem = sensor;
    this.angleToRotate = angleToRotate;

    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    currentAngle = m_robotDrive.getTotalRotation();
    targetAngle = currentAngle + angleToRotate;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentAngle = m_robotDrive.getTotalRotation();
    rotationDifference = targetAngle - currentAngle;

    NetworkTableInstance.getDefault().getTable("AutoTurn").getEntry("angleDifference").setDouble(rotationDifference);
    NetworkTableInstance.getDefault().getTable("AutoTurn").getEntry("rotation").setDouble(rot);

    if (rotationDifference > 25) rot = 0.3;
    else if (rotationDifference < -25) rot = -0.3;
    else if (rotationDifference > 0) rot = rotationDifference * 0.013 + 0.015;
    else if (rotationDifference < 0) rot = rotationDifference * 0.013 - 0.015;

    rot *= -1;

    m_robotDrive.drive(0, 0, rot, false, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(rotationDifference) < 3;
  }
}

