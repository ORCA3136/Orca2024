// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TestSparkSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  CANSparkMax m_Spark1;
  AbsoluteEncoder encoder;

  public TestSparkSubsystem() {

    m_Spark1 = new CANSparkMax(Constants.DriveConstants.kTestSparkCanId, MotorType.kBrushless);
    m_Spark1.setIdleMode(IdleMode.kCoast);

    encoder = m_Spark1.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("AbsoluteEncoderPosition").setDouble(encoder.getPosition());
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("AbsoluteEncoderVelocity").setDouble(encoder.getVelocity());
  }

  public void RunIntake(double speed) {
    
  }

}
