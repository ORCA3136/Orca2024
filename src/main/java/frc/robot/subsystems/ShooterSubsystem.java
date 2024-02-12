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

public class ShooterSubsystem extends SubsystemBase {
  
  CANSparkMax m_ShooterLeft;
  CANSparkMax m_ShooterRight;

  SparkPIDController rightPid;
  SparkPIDController leftPid;

  private double setPoint = 0;
  private boolean usePID = false;

  public ShooterSubsystem() {

    m_ShooterRight = new CANSparkMax(Constants.DriveConstants.kShooterRightCanId, MotorType.kBrushless);
    m_ShooterRight.setIdleMode(IdleMode.kCoast);

    m_ShooterLeft = new CANSparkMax(Constants.DriveConstants.kShooterLeftCanId, MotorType.kBrushless);
    m_ShooterLeft.setIdleMode(IdleMode.kCoast);

    // Negative is forward
    rightPid = m_ShooterRight.getPIDController();
    rightPid.setP(0.0003);
    rightPid.setI(0.000001);
    rightPid.setD(0.0);
    rightPid.setFF(0.0001875);
    rightPid.setIZone(100);
    rightPid.setOutputRange(0.0, 0.0);
    m_ShooterRight.burnFlash();

    // Positive is forward
    leftPid = m_ShooterLeft.getPIDController();
    leftPid.setP(0.0003);
    leftPid.setI(0.000001);
    leftPid.setD(0.0);
    leftPid.setFF(0.0001875);
    leftPid.setIZone(100);
    leftPid.setOutputRange(0.0, 0.0);
    m_ShooterLeft.burnFlash();

  }

  @Override
  public void periodic() {
    if (usePID) {
      leftPid.setReference(setPoint, ControlType.kVelocity);
      rightPid.setReference(-setPoint, ControlType.kVelocity);
    }
    NetworkTableInstance.getDefault().getTable("Shooter").getEntry("RPM").setDouble(getSpeed());
  }

  public void RunShooter(double speed) {
    m_ShooterRight.set(speed);
    m_ShooterLeft.set(-speed);
  }

  private double getSpeed() {
    return m_ShooterRight.getEncoder().getVelocity();
  }

  private double getError(){
    return setPoint-getSpeed();
  }

  private boolean atSpeed(){
    return Math.abs(getError()) < 100;
  }

  public Command shootNote() {
    return startEnd(
      () -> setNewTarget(Constants.ShooterConstants.shoot),
      () -> setStopTarget()
      );
  }

  public Command reverseShootNote() {
    return startEnd(
      () -> setNewTarget(Constants.ShooterConstants.reverse),
      () -> setStopTarget()
      );
  }

  private void setNewTarget(double setPoint) {
    if (setPoint > 0) {
      rightPid.setOutputRange(-1.0, 0.0);
      leftPid.setOutputRange(0.0, 1.0);
      this.setPoint = setPoint;
    }
    else if (setPoint < 0) {
      rightPid.setOutputRange(0.0, 1.0);
      leftPid.setOutputRange(-1.0, 0.0);
      this.setPoint = setPoint;
    }
  }

  private void setStopTarget() {
    rightPid.setOutputRange(0.0, 0.0);
    leftPid.setOutputRange(0.0, 0.0);
    this.setPoint = 0;
  }
}
