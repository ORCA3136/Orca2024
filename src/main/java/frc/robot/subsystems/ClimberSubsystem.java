// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CurrentConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  CANSparkMax m_ClimberLeft;
  CANSparkMax m_ClimberRight;

  public ClimberSubsystem() {

    m_ClimberLeft = new CANSparkMax(Constants.DriveConstants.kClimberLeftCanId, MotorType.kBrushless);
    m_ClimberLeft.restoreFactoryDefaults();
    m_ClimberLeft.setSmartCurrentLimit(CurrentConstants.AMP40, CurrentConstants.AMP30);
    m_ClimberLeft.setIdleMode(IdleMode.kBrake);
    m_ClimberLeft.setInverted(true);
    m_ClimberLeft.burnFlash();

    m_ClimberRight = new CANSparkMax(Constants.DriveConstants.kClimberRightCanId, MotorType.kBrushless);
    m_ClimberRight.restoreFactoryDefaults();
    m_ClimberRight.setIdleMode(IdleMode.kBrake);
    m_ClimberRight.setSmartCurrentLimit(CurrentConstants.AMP40, CurrentConstants.AMP30);
    m_ClimberRight.burnFlash();

   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command RunClimber(double speed) {
    return runOnce(() -> { m_ClimberRight.set(speed); m_ClimberLeft.set(speed); });
  }
}
