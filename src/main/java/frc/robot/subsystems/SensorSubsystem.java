// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
//import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
//import edu.wpi.first.wpilibj.DataLogManager;
//import edu.wpi.first.util.datalog.BooleanLogEntry;
//import edu.wpi.first.util.datalog.DataLog;
//import edu.wpi.first.util.datalog.DoubleLogEntry;
//import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SensorSubsystem extends SubsystemBase {

  private static DigitalInput DIO_0;

  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private Pose2d robotPose2d;
  private DriverStation.Alliance alliance = DriverStation.getAlliance().get();
  private boolean output0;
  private DriveSubsystem robotDrive;

  /** Creates a new SensorSubsystem. */
  public SensorSubsystem(DriveSubsystem drive) {

    DIO_0 = new DigitalInput(0);
    robotDrive = drive;

  }

  @Override
  public void periodic() {
    
    output0 = DIO_0.get();
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("DIO_0").setBoolean(output0);
    
    if (LimelightHelpers.getTV("limelight")) {
      robotDrive.resetOdometry(LimelightHelpers.getBotPose2d_wpiBlue("limelight"));
    }

  }
}
