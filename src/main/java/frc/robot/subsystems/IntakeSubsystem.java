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

  CANSparkMax m_LeftArm;
  CANSparkMax m_RightArm;

  CANSparkMax m_Intake;
  CANSparkMax m_Shooter;

  public IntakeSubsystem() {

    m_LeftArm = new CANSparkMax(Constants.DriveConstants.kLeftArmCanId, MotorType.kBrushless);
    m_RightArm = new CANSparkMax(Constants.DriveConstants.kRightArmCanId, MotorType.kBrushless);

    m_LeftArm.setIdleMode(IdleMode.kBrake);
    m_RightArm.setIdleMode(IdleMode.kBrake);

    m_RightArm.setInverted(true);

    //m_Intake = new CANSparkMax(Constants.DriveConstants.kIntakeCanId, MotorType.kBrushless);
    //m_Intake.setIdleMode(IdleMode.kCoast);

    //m_Shooter = new CANSparkMax(Constants.DriveConstants.kShooterCanId, MotorType.kBrushless);
    //m_Shooter.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void RunIntake(double speed) {
    m_Intake.set(speed);
  }

  public void RunShooter(double speed) {
    m_Shooter.set(speed);
  }

  public void RunArm(double speed) {
    m_LeftArm.set(speed);
    m_RightArm.set(speed);
  }

}
