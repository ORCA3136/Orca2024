package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {


    public void setDesiredState(SwerveModuleState state);

    public void resetEncoders();

    public SwerveModulePosition getPosition();

    public SwerveModuleState getState();



    public void setPID(double p, double i, double d);


    
}
