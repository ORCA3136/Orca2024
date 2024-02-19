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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  CANSparkMax m_LeftArm;
  CANSparkMax m_RightArm;

  AbsoluteEncoder encoder;

  double targetPosition;
  boolean shootNoteInUse = false;

  double kS = 0;
  double kG = 0.6;
  double kV = 3.61;
  double kA = 0.04;

  double kP = Constants.ArmPIDConstants.armkP;
  double kI = Constants.ArmPIDConstants.armkI;
  double kD = Constants.ArmPIDConstants.armkD;

  ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);
  PIDController pidController = new PIDController(kP, kI, kD);

  double setpoint = -1;

  public ArmSubsystem(RobotContainer robot) {

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("S Gain", kS);
    SmartDashboard.putNumber("G Gain", kG);
    SmartDashboard.putNumber("V Gain", kV);
    SmartDashboard.putNumber("A Gain", kA);
    SmartDashboard.putNumber("Arm Setpoint", setpoint);

    robotContainer = robot;

    //Left arm spark has absolute encoder
    m_LeftArm = new CANSparkMax(Constants.DriveConstants.kLeftArmCanId, MotorType.kBrushless);
    m_RightArm = new CANSparkMax(Constants.DriveConstants.kRightArmCanId, MotorType.kBrushless);

    m_LeftArm.setIdleMode(IdleMode.kBrake);
    m_RightArm.setIdleMode(IdleMode.kBrake);

    m_RightArm.follow(m_LeftArm, true);

    //double LeftIntakeVoltage = LeftIntake.getBusVoltage();

    encoder = m_LeftArm.getAbsoluteEncoder(Type.kDutyCycle);

    pidController.setTolerance(Constants.ArmPIDConstants.kPositionTolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("AbsoluteEncoderPosition").setDouble(getDistance());
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("TargetSetpoint").setDouble(setpoint);
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("AtSetpoint").setBoolean(pidController.atSetpoint());

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double s = SmartDashboard.getNumber("S Gain", 0);
    double g = SmartDashboard.getNumber("G Gain", 0);
    double v = SmartDashboard.getNumber("V Gain", 0);
    double a = SmartDashboard.getNumber("A Gain", 0);
    double set = SmartDashboard.getNumber("Arm Setpoint", 0);

    /*
    if (kP != p || kI != i || kD != d) {
      kP = p; kI = i; kD = d;
      pidController = new PIDController(kP, kI, kD);
    }*/
    if (kS != s || kG != g || kV != v || kA != a) {
      kS = s; kG = g; kV = v; kA = a;
      feedforward = new ArmFeedforward(kS, kG, kV);
    }
    
    setpoint = robotContainer.getLeftTrigger() * 0.5;
    
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("SetVoltage").setDouble(feedforward.calculate(setpoint, kV, 0));
    //if (setpoint != -1)
    m_LeftArm.setVoltage(
    pidController.calculate(getDistance(), setpoint) 
    //+ 
    //feedforward.calculate(setpoint - 0.02, 0, 0)
    );
  }

  public Command RunArmPID(double setpoint) {
    return runOnce(() -> { this.setpoint = setpoint; if (setpoint == -1) m_LeftArm.set(0);});
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