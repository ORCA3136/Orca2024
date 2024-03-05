// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CurrentConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  CANSparkMax m_ClimberLeft;
  CANSparkMax m_ClimberRight;

  RelativeEncoder m_RightEncoder;
  RelativeEncoder m_LeftEncoder;

  public ClimberSubsystem() {

    m_ClimberLeft = new CANSparkMax(Constants.DriveConstants.kClimberLeftCanId, MotorType.kBrushless);
    //m_ClimberLeft.restoreFactoryDefaults();
    m_ClimberLeft.setSmartCurrentLimit(CurrentConstants.AMP40, CurrentConstants.AMP30);
    m_ClimberLeft.setIdleMode(IdleMode.kBrake);
    m_ClimberLeft.setInverted(true);
    //m_ClimberLeft.burnFlash();

    m_ClimberRight = new CANSparkMax(Constants.DriveConstants.kClimberRightCanId, MotorType.kBrushless);
    //m_ClimberRight.restoreFactoryDefaults();
    m_ClimberRight.setIdleMode(IdleMode.kBrake);
    m_ClimberRight.setSmartCurrentLimit(CurrentConstants.AMP40, CurrentConstants.AMP30);
    //m_ClimberRight.burnFlash();

    m_RightEncoder = m_ClimberRight.getEncoder();
    m_LeftEncoder = m_ClimberLeft.getEncoder();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    NetworkTableInstance.getDefault().getTable("Climber").getEntry("LeftPos").setDouble(m_LeftEncoder.getPosition());
    NetworkTableInstance.getDefault().getTable("Climber").getEntry("RightPos").setDouble(m_RightEncoder.getPosition());

    if (m_RightEncoder.getPosition() < -225) { m_ClimberRight.set(0.1); }
    if (m_LeftEncoder.getPosition() < -225) { m_ClimberLeft.set(0.1); }
  }

  public Command RunClimber(double speed) {
    return runOnce(() -> { 
      m_ClimberRight.set(speed); m_ClimberLeft.set(speed); 
    });
  }

  public Command ResetEncoders() {
    return runOnce(() -> {
      m_RightEncoder.setPosition(0);
      m_LeftEncoder.setPosition(0);
    });
  }
}
