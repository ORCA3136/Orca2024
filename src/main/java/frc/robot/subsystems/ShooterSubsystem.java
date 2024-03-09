// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CurrentConstants;

public class ShooterSubsystem extends SubsystemBase {
  
  CANSparkMax m_ShooterLeft;
  CANSparkMax m_ShooterRight;

  SparkPIDController rightPid;
  SparkPIDController leftPid;

  private double setPoint = 0;

  SensorSubsystem sensorSubsystem;

  public ShooterSubsystem(SensorSubsystem sensor) {

    sensorSubsystem = sensor;

    m_ShooterRight = new CANSparkMax(Constants.DriveConstants.kShooterRightCanId, MotorType.kBrushless);
    //m_ShooterRight.restoreFactoryDefaults();
    m_ShooterRight.setIdleMode(IdleMode.kCoast);
    m_ShooterRight.setSmartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);
    m_ShooterRight.setInverted(true);


    m_ShooterLeft = new CANSparkMax(Constants.DriveConstants.kShooterLeftCanId, MotorType.kBrushless);
    //m_ShooterLeft.restoreFactoryDefaults();
    m_ShooterLeft.setIdleMode(IdleMode.kCoast);
    m_ShooterLeft.setSmartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);


    m_ShooterRight.getEncoder().setMeasurementPeriod(16);
    m_ShooterLeft.getEncoder().setMeasurementPeriod(16);
    m_ShooterRight.getEncoder().setAverageDepth(2);
    m_ShooterLeft.getEncoder().setAverageDepth(2);


    // Negative is forward
    rightPid = m_ShooterRight.getPIDController();
    rightPid.setP(0.0003);
    rightPid.setI(0.000001);
    rightPid.setD(0.0);
    rightPid.setFF(0.0001875);
    rightPid.setIZone(100);
    rightPid.setOutputRange(0.0, 0.0);
    //m_ShooterRight.burnFlash();

    // Positive is forward
    leftPid = m_ShooterLeft.getPIDController();
    leftPid.setP(0.0003);
    leftPid.setI(0.000001);
    leftPid.setD(0.0);
    leftPid.setFF(0.0001875);
    leftPid.setIZone(100);
    leftPid.setOutputRange(0.0, 0.0);
    //m_ShooterLeft.burnFlash();
  }

  @Override
  public void periodic() {


    leftPid.setReference(setPoint, ControlType.kVelocity);
    rightPid.setReference(setPoint, ControlType.kVelocity);

    if (sensorSubsystem.onSide()) {
      if (setPoint == 0 && !sensorSubsystem.getIntakeSensor(0)) {
        setNewTarget(1100);
      }
      if (setPoint == 1100 && sensorSubsystem.getIntakeSensor(0)) {
        setNewTarget(0);
      }
    }
    else {
      if (setPoint == 1100) {
        setNewTarget(0);
      }
    }

    NetworkTableInstance.getDefault().getTable("Shooter").getEntry("RPM").setDouble(getSpeed());
    NetworkTableInstance.getDefault().getTable("Shooter").getEntry("Target RPM").setDouble(setPoint);
    NetworkTableInstance.getDefault().getTable("Shooter").getEntry("RPM Error").setDouble(getError());
  }

  public void RunShooter(double speed) {
    m_ShooterRight.set(-speed);
    m_ShooterLeft.set(-speed);
  }

  public double getSpeed() {
    return m_ShooterRight.getEncoder().getVelocity();
  }

  private double getError() {
    return setPoint-getSpeed();
  }

  public boolean atSpeed() {
    return Math.abs(getError()) < 100;
  }

  public Command shootNote(double target) {
    return runOnce(
      () -> setNewTarget(target)
      );
  }

  public void setShootSpeed(double target) {
    setNewTarget(target);
  }

  public Command shootNoteNOTNOTSensor(SensorSubsystem sensor) {
    return runOnce(
      () -> setNewTarget(sensor.speedMap)
      );
  }

  public void setNewTarget(double setPoint) {
    if (setPoint > 0) {
      rightPid.setOutputRange(0.0, 1.0);
      leftPid.setOutputRange(0.0, 1.0);
      this.setPoint = setPoint;
    }
    else if (setPoint < 0) {
      rightPid.setOutputRange(-1.0, 0.0);
      leftPid.setOutputRange(-1.0, 0.0);
      this.setPoint = setPoint;
    }
    else setStopTarget();
  }

  private void setStopTarget() {
    rightPid.setOutputRange(0.0, 0.0);
    leftPid.setOutputRange(0.0, 0.0);
    this.setPoint = 0;
  }

  public double getTargetSpeed() {
    return setPoint;
  }
}
