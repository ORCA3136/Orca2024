package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class ForwardClimb extends Command {
    
    ArmSubsystem m_ArmSubsystem;
    ClimberSubsystem m_ClimberSubsystem;

    double armPosition;
    double climberLeftPosition;
    double climberRightPosition;

    double armThreshold = 85;
    double climberHeight = -110;

    public ForwardClimb(ArmSubsystem arm, ClimberSubsystem climber) {
        m_ArmSubsystem = arm;
        m_ClimberSubsystem = climber;
        addRequirements(arm, climber);
    }

    @Override
    public void initialize() {

        m_ArmSubsystem.SetPositionPID(95);

    }

    @Override
  public void execute() {

    armPosition = m_ArmSubsystem.getDistance();
    climberLeftPosition = m_ClimberSubsystem.getLeftPos();
    climberRightPosition = m_ClimberSubsystem.getRightPos();
    
    NetworkTableInstance.getDefault().getTable("ClimberSequence").getEntry("ArmThreshold").setBoolean(armPosition > armThreshold);
    NetworkTableInstance.getDefault().getTable("ClimberSequence").getEntry("LeftThreshold").setBoolean(climberLeftPosition > climberHeight);
    NetworkTableInstance.getDefault().getTable("ClimberSequence").getEntry("RightThreshold").setBoolean(climberRightPosition > climberHeight);

    if (armPosition > armThreshold) {

        if (climberLeftPosition > climberHeight) {
            m_ClimberSubsystem.RunLeftClimber(-0.3);
        }
        else m_ClimberSubsystem.RunLeftClimber(0);

        if (climberRightPosition > climberHeight) {
            m_ClimberSubsystem.RunRightClimber(-0.3);
        }
        else m_ClimberSubsystem.RunRightClimber(0);

    }
    else {
        m_ClimberSubsystem.RunLeftClimber(0);
        m_ClimberSubsystem.RunRightClimber(0);
    }

  }

  @Override
  public void end(boolean interrupted) {

    m_ClimberSubsystem.RunLeftClimber(0);
    m_ClimberSubsystem.RunRightClimber(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
