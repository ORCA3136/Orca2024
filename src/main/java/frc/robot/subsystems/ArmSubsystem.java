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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmPID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  RobotContainer robotContainer;

  ParallelCommandGroup shootNote;

  CANSparkMax m_LeftArm;
  CANSparkMax m_RightArm;

  AbsoluteEncoder encoder;

  double targetPosition;
  boolean useTrigger = true;
  boolean shootNoteInUse = false;

  double kS = 0;
  double kG = 0;
  double kV = 0;
  double kA = 0;

  ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);
  PIDController pidController = new PIDController(Constants.ArmPIDConstants.armkP, Constants.ArmPIDConstants.armkI, Constants.ArmPIDConstants.armkD);

  double setpoint = -1;

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

    shootNote = robotContainer.getShootNote();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("AbsoluteEncoderPosition").setDouble(encoder.getPosition());
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("AbsoluteEncoderVelocity").setDouble(encoder.getVelocity());
    
    /*if (setpoint != -1)
    m_LeftArm.setVoltage(pidController.calculate(getDistance(), setpoint) 
    + feedforward.calculate(setpoint, Constants.ArmPIDConstants.armVelocity, Constants.ArmPIDConstants.armAcceleration)
    );*/
  }

  public Command RunArmPID(double setpoint) {
    return runOnce(() -> { this.setpoint = setpoint; if (setpoint == -1) m_LeftArm.set(0);});
  }

  public void RunArm(double speed) {
    m_LeftArm.set(speed);
  }

  public double getDistance()
  {
    return encoder.getPosition() * 10;
  }

  public double getTargetPosition() {
    return 0.0;
  }
}

// Calculates the feedforward for a position of 1 units, a velocity of 2 units/second, and
// an acceleration of 3 units/second^2
// Units are determined by the units of the gains passed in at construction.
////////feedforward.calculate(1, 2, 3);