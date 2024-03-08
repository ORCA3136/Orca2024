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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.CurrentConstants;
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

  double kS = 0.0;
  double kG = 0.0175;
  double kV = 0.0; // 3.61
  double kA = 0.0; // 0.04

  double kP = Constants.ArmPIDConstants.armkP;
  double kD = Constants.ArmPIDConstants.armkD;

  SparkPIDController pidController;

  double feedforward = 0;
  double setpoint = -1;

  double tempSetpoint = -1;

  public ArmSubsystem(RobotContainer robot) {

    robotContainer = robot;

    SmartDashboard.putNumber("kG", kG);
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kD", kD);

    //Left arm spark has absolute encoder
    m_LeftArm = new CANSparkMax(Constants.DriveConstants.kLeftArmCanId, MotorType.kBrushless);
    m_RightArm = new CANSparkMax(Constants.DriveConstants.kRightArmCanId, MotorType.kBrushless);

    //m_LeftArm.restoreFactoryDefaults();
    //m_RightArm.restoreFactoryDefaults();
    m_LeftArm.setSmartCurrentLimit(CurrentConstants.AMP40, CurrentConstants.AMP25);
    m_RightArm.setSmartCurrentLimit(CurrentConstants.AMP40,CurrentConstants.AMP25);

    m_LeftArm.setIdleMode(IdleMode.kBrake);
    m_RightArm.setIdleMode(IdleMode.kBrake);

    m_RightArm.follow(m_LeftArm, true);

    //m_LeftArm.burnFlash();
    //m_RightArm.burnFlash();

    //double LeftIntakeVoltage = LeftIntake.getBusVoltage();

    pidController = m_LeftArm.getPIDController();
    pidController.setP(kP);
    pidController.setI(0);
    pidController.setD(kD);

    pidController.setPositionPIDWrappingEnabled(true);
    pidController.setPositionPIDWrappingMinInput(0.0);
    pidController.setPositionPIDWrappingMaxInput(360);

    encoder = m_LeftArm.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setPositionConversionFactor(360);
    pidController.setFeedbackDevice(encoder);

    // Create a new ArmFeedforward with gains kS, kG, kV, and kA
    ArmFeedforward armFeedforward = new ArmFeedforward(kS, kG, kV, kA);

    // Calculates the feedforward for a position of 1 units, a velocity of 2 units/second, and
    // an acceleration of 3 units/second^2
    // Units are determined by the units of the gains passed in at construction.
    armFeedforward.calculate(1, 2, 3);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("AbsoluteEncoderPosition").setDouble(getDistance());
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("TargetSetpoint").setDouble(setpoint);
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("TempTargetSetpoint").setDouble(tempSetpoint);
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("MotorVelocity").setDouble(encoder.getVelocity());

    // 2.5 Floor Pickup
    // 29 Under Stage
    // 55 Safe position
    // 71 Source - 69 actual position
    // 92 In line with edge of bumpers --- Amp


    kG = SmartDashboard.getNumber("kG", kG);
    kV = SmartDashboard.getNumber("kV", kV);


    if (kP != SmartDashboard.getNumber("kP", kP) || kD != SmartDashboard.getNumber("kD", kD)) {
      kP = SmartDashboard.getNumber("kP", kP);
      kD = SmartDashboard.getNumber("kD", kD);
      pidController.setP(kP);
      pidController.setD(kD);
    }
    if (setpoint == -1) setpoint = -1;
    else if (setpoint < 1) { setpoint = 1; tempSetpoint = 1; } 
    else if (setpoint > 100) { setpoint = 100; tempSetpoint = 1; } 

    // Horizontal angle 5
    // Vertical angle 95
    if (setpoint != -1) {


      if (getError() < 5 && encoder.getVelocity() < 0.002) tempSetpoint = tempSetpoint + 0.05*(setpoint - getDistance());

      if (getError() > 5) tempSetpoint = setpoint;

      if (Math.abs(tempSetpoint - setpoint) > 7) tempSetpoint = setpoint;

      // P increases when setpoint is low
      double p = kP * 0.3 + kP * 0.6 * Math.abs(Math.cos((getDistance() + 5) * (Math.PI/180)));
      // P increases when going up
      if (setpoint > getDistance()) p += kP * 0.2;
      // P decreases when difference in setpoints is large
      double diff = Math.abs(setpoint - getDistance());
      if (diff > 60) p *= 0.5;
      else if (diff > 30) p *= 0.8;
      pidController.setP(p);

      feedforward = kG * ((getDistance() + 5) * (Math.PI/180));
      pidController.setReference(tempSetpoint, ControlType.kPosition, 0, feedforward);
      NetworkTableInstance.getDefault().getTable("Arm").getEntry("PIDWorking?").setString("True" + RobotController.getFPGATime());
    }
    else {
      NetworkTableInstance.getDefault().getTable("Arm").getEntry("PIDWorking?").setString("False" + RobotController.getFPGATime()); 
    }

    // Arm Code Hardstops
    if (encoder.getPosition() > Constants.ArmStops.StopPostion) m_LeftArm.set(Constants.ArmStops.StopSpeed);

    if (encoder.getPosition() > Constants.ArmStops.BackPostion) m_LeftArm.set(Constants.ArmStops.BackSpeed);

  }

  public Command SetPIDPosition(double setpoint) {
    return runOnce(() -> { 
      this.setpoint = setpoint; 
      tempSetpoint = setpoint;
    });
  }

  public Command SetPIDNOTNOTSensor(SensorSubsystem sensor) {
    return runOnce(() -> { 
      this.setpoint = sensor.angleMap;
      tempSetpoint = setpoint;
    });
  }

  public Command RunArm(double speed) {
    return runOnce(() -> {
      setpoint = -1;
      m_LeftArm.set(speed);
    });
  }

  public double getDistance()
  {
    return encoder.getPosition();
  }

  public double getError() 
  {
    return Math.abs(setpoint - getDistance());
  }

  public double getTargetPosition() {
    return 0.0;
  }

  public boolean checkUserButton() {
    return false;
    //RobotController.getUserButton();
  }

  public void setArmModeWhileDisabled() {


    m_LeftArm.setIdleMode(IdleMode.kCoast);
    m_RightArm.setIdleMode(IdleMode.kCoast);


    m_LeftArm.setIdleMode(IdleMode.kBrake);
    m_RightArm.setIdleMode(IdleMode.kBrake);
  }

  public Command RunkG() {
    return runOnce(() -> {
      setpoint = -1;
      m_LeftArm.set(feedforward);
    });
  }
}

// Calculates the feedforward for a position of 1 units, a velocity of 2 units/second, and
// an acceleration of 3 units/second^2
// Units are determined by the units of the gains passed in at construction.
////////feedforward.calculate(1, 2, 3);