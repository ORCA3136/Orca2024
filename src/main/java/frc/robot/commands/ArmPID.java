package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPID extends PIDCommand {
  /** Creates a new ReplaceMePIDCommand. */
  public ArmPID(ArmSubsystem arm) {
    super(
        // The controller that the command will use
        new PIDController(Constants.ArmPIDConstants.armkP, Constants.ArmPIDConstants.armkI, Constants.ArmPIDConstants.armkD),
        // This should return the measurement
        arm::getDistance,
        // This should return the setpoint (can also be a constant)
        arm::getTargetPosition,
        // This uses the output
        output-> arm.RunArm(output * Constants.ArmPIDConstants.pidThrottle),
        arm);
        getController().setTolerance(Constants.ArmPIDConstants.kPositionTolerance);
          // Use the output here
        }
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}