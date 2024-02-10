// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmPID;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  RobotContainer robotContainer;

  CANSparkMax m_LeftArm;
  CANSparkMax m_RightArm;

  AbsoluteEncoder encoder;

  double targetPosition;
  boolean useTrigger = true;
  boolean PIDinUse = false;

  public ArmSubsystem(RobotContainer robot) {

    robotContainer = robot;

    //Left arm spark has absolute encoder
    m_LeftArm = new CANSparkMax(Constants.DriveConstants.kLeftArmCanId, MotorType.kBrushless);
    m_RightArm = new CANSparkMax(Constants.DriveConstants.kRightArmCanId, MotorType.kBrushless);

    m_LeftArm.setIdleMode(IdleMode.kBrake);
    m_RightArm.setIdleMode(IdleMode.kBrake);

    m_RightArm.follow(m_LeftArm, true);

    //double LeftIntakeVoltage = LeftIntake.getBusVoltage();

    encoder = m_LeftArm.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("AbsoluteEncoderPosition").setDouble(encoder.getPosition());
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("AbsoluteEncoderVelocity").setDouble(encoder.getVelocity());
  }

  public void RunArm(double speed) {
    m_LeftArm.set(speed);
  }

  public double getDistance()
  {
    return encoder.getPosition() * 50;
  }

  public double getTargetPosition() {
    
    if (robotContainer.getPOV() != -1 && !PIDinUse) { robotContainer.StartPID(); PIDinUse = true; }

    if (robotContainer.getPOV() == 0) { targetPosition = 0.48; }
    else if (robotContainer.getPOV() == 180) { targetPosition = 0.03; }
    else if (robotContainer.getPOV() == 90 || robotContainer.getPOV() == -90) { targetPosition = 0.10; }
    else if (targetPosition == 0) { targetPosition = 0.10; }

    //0.48 Amp --- Intake level 0.3 --- Intake low 0.15 --- 0.03 Floor

    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("TargetPosition").setDouble(targetPosition);

    return targetPosition * 50;
  }

  public void setTargetPosition(double position) 
  { 
    targetPosition = position; 
  }

  public void setInUse() {
    PIDinUse = false;
  }
}
