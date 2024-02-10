// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  
  CANSparkMax m_ShooterLeft;
  CANSparkMax m_ShooterRight;

  PIDController m_LeftPID;

  public ShooterSubsystem() {

    m_ShooterLeft = new CANSparkMax(Constants.DriveConstants.kShooterLeftCanId, MotorType.kBrushless);
    m_ShooterLeft.setIdleMode(IdleMode.kCoast);

    m_ShooterRight = new CANSparkMax(Constants.DriveConstants.kShooterRightCanId, MotorType.kBrushless);
    m_ShooterRight.setIdleMode(IdleMode.kCoast);

    m_ShooterLeft.follow(m_ShooterRight, true);

    m_LeftPID = new PIDController(0.0, 0.0, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void RunShooter(double speed) {
    m_ShooterRight.set(speed);
  }
}
