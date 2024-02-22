// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.CurrentConstants;
import frc.robot.commands.ArmPID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  RobotContainer robotContainer;

  CANSparkMax m_LeftArm;
  CANSparkMax m_RightArm;

  AbsoluteEncoder encoder;

  double targetPosition;
  boolean shootNoteInUse = false;
  boolean useTrigger = false;

  double kS = 0;
  double kG = 0.6;
  double kV = 3.61;
  double kA = 0.04;

  double kP = Constants.ArmPIDConstants.armkP;
  double kI = Constants.ArmPIDConstants.armkI;
  double kD = Constants.ArmPIDConstants.armkD;

  ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);
  SparkPIDController pidController;

  double setpoint = -1;

  public ArmSubsystem(RobotContainer robot) {

    robotContainer = robot;

    

    //Left arm spark has absolute encoder
    m_LeftArm = new CANSparkMax(Constants.DriveConstants.kLeftArmCanId, MotorType.kBrushless);
    m_RightArm = new CANSparkMax(Constants.DriveConstants.kRightArmCanId, MotorType.kBrushless);

    m_LeftArm.restoreFactoryDefaults();
    m_RightArm.restoreFactoryDefaults();
    m_LeftArm.setSmartCurrentLimit(CurrentConstants.AMP30, CurrentConstants.AMP25);
    m_RightArm.setSmartCurrentLimit(CurrentConstants.AMP30,CurrentConstants.AMP25);

    m_LeftArm.setIdleMode(IdleMode.kBrake);
    m_RightArm.setIdleMode(IdleMode.kBrake);

    m_RightArm.follow(m_LeftArm, true);

    m_LeftArm.burnFlash();
    m_RightArm.burnFlash();

    //double LeftIntakeVoltage = LeftIntake.getBusVoltage();

    pidController = m_LeftArm.getPIDController();
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);

    pidController.setPositionPIDWrappingEnabled(true);
    pidController.setPositionPIDWrappingMinInput(0.0);
    pidController.setPositionPIDWrappingMaxInput(360);

    encoder = m_LeftArm.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setPositionConversionFactor(360);
    pidController.setFeedbackDevice(encoder);
    pidController.setOutputRange(-0.3, 0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("AbsoluteEncoderPosition").setDouble(getDistance());
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("TargetSetpoint").setDouble(setpoint);
    


    if (useTrigger) {
      setpoint = robotContainer.getLeftTrigger() * 0.9 + 0.04;
      setpoint *= 90;
    }
    
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("SetVoltage").setDouble(feedforward.calculate(setpoint, kV, 0));
    
    
    pidController.setReference(setpoint, ControlType.kPosition);
  }

  public Command SetPIDPosition(double setpoint) {
    return runOnce(() -> { 
      this.setpoint = setpoint; 
    });
  }

  public Command SetPIDNOTNOTSensor(SensorSubsystem sensor) {
    return runOnce(() -> { 
      this.setpoint = sensor.angleMap; 
    });
  }

  public void RunArm(double speed) {
    m_LeftArm.set(speed);
  }

  public double getDistance()
  {
    return encoder.getPosition();
  }

  public double getTargetPosition() {
    return 0.0;
  }
}

// Calculates the feedforward for a position of 1 units, a velocity of 2 units/second, and
// an acceleration of 3 units/second^2
// Units are determined by the units of the gains passed in at construction.
////////feedforward.calculate(1, 2, 3);