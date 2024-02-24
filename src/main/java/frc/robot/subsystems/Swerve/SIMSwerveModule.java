package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SIMSwerveModule implements SwerveModuleIO {


    DCMotorSim m_driveMotor;
    DCMotorSim m_turningMotor;
    PIDController m_drivePIDController;
    PIDController m_turningPIDController;
    double DriveConversionFactor = 1/(6.12/ ((Units.inchesToMeters(4) * Math.PI) ));

    public SIMSwerveModule() {
        m_driveMotor = new DCMotorSim(DCMotor.getKrakenX60(1), 6.12, 0.04);
        m_turningMotor = new DCMotorSim(DCMotor.getFalcon500(1), 6.12, 0.04);
        m_drivePIDController = new PIDController(10, 0, 0);
        m_turningPIDController = new PIDController(1, 0, 0);
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }


    @Override
    public void setDesiredState(SwerveModuleState state) {
        // System.out.println(state);
        m_driveMotor.update(0.02);
        m_turningMotor.update(0.02);
        m_driveMotor.setInputVoltage(m_drivePIDController.calculate(getSpeedMPS(), state.speedMetersPerSecond));
        m_turningMotor.setInputVoltage(m_turningPIDController.calculate(getAngle().getRadians(), state.angle.getRadians()));
    }

    private double getSpeedMPS() {
        return m_driveMotor.getAngularVelocityRadPerSec()* DriveConversionFactor;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_turningMotor.getAngularPositionRad() % (2 * Math.PI));

    }

    @Override
    public void resetEncoders() {
        m_driveMotor.setState(0, 0);
        m_turningMotor.setState(0, 0);
    }

    @Override
    public SwerveModulePosition getPosition() { 
        return new SwerveModulePosition(m_driveMotor.getAngularPositionRad() * DriveConversionFactor, getAngle());
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeedMPS(), getAngle());
    }

    @Override
    public void setPID(double p, double i, double d) {
        m_drivePIDController.setPID(p, i, d);
    }

    
    
}
