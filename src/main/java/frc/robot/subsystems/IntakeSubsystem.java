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

  CANSparkMax m_LeftIntake;
  CANSparkMax m_RightIntake;

  public IntakeSubsystem() {

    m_LeftIntake = new CANSparkMax(Constants.DriveConstants.kLeftIntakeCanId, MotorType.kBrushless);
    m_RightIntake = new CANSparkMax(Constants.DriveConstants.kRightIntakeCanId, MotorType.kBrushless);

    m_LeftIntake.setIdleMode(IdleMode.kCoast);
    m_RightIntake.setIdleMode(IdleMode.kCoast);

    m_RightIntake.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void RunIntake(double speed) {

    m_LeftIntake.set(speed);
    m_RightIntake.set(speed);

  }

}
