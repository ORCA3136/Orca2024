// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  CANSparkMax m_IntakeLeft;
  CANSparkMax m_IntakeRight;

  public IntakeSubsystem() {

    m_IntakeLeft = new CANSparkMax(Constants.DriveConstants.kIntakeLeftCanId, MotorType.kBrushless);
    m_IntakeLeft.setIdleMode(IdleMode.kCoast);

    m_IntakeRight = new CANSparkMax(Constants.DriveConstants.kIntakeRightCanId, MotorType.kBrushless);
    m_IntakeRight.setIdleMode(IdleMode.kCoast);

    m_IntakeLeft.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void RunIntake(double speed) {
    m_IntakeRight.set(speed);
    m_IntakeLeft.set(speed);
  }
}