// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DataLogManager;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Time;

public class SensorSubsystem extends SubsystemBase {

  private double angleForArm;
  private double speedForShooter;

  private static DigitalInput DIO_0;
  private static DigitalInput DIO_1;

  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private Pose2d robotPose2d;
  //private DriverStation.Alliance alliance = DriverStation.getAlliance().get();
  private boolean output0;
  private boolean output1;
  private boolean[] sensorValues;
  private DriveSubsystem robotDrive;

  private InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();

  // True means note is in correct position
  private boolean IntakeState = false;
  

    boolean red;
    Pose2d pose;
    Translation2d speaker = Constants.Field.BLUE_SPEAKER_FROM_CENTER;
    double angle;
    
    double xDistance;
    double yDistance;

    double distanceToSpeaker;
    double angleToSpeaker;

    public double speedMap;
    public double angleMap;

  /** Creates a new SensorSubsystem. */
  public SensorSubsystem(DriveSubsystem drive) {

    LimelightHelpers.setPipelineIndex("limelight-note", 0);

    DIO_0 = new DigitalInput(0);
    DIO_1 = new DigitalInput(1);
    robotDrive = drive;

    sensorValues = new boolean[2];

    shooterSpeedMap.put(Double.valueOf(1.27), Double.valueOf(2600.0));
    shooterSpeedMap.put(Double.valueOf(1.88), Double.valueOf(2850.0));
    shooterSpeedMap.put(Double.valueOf(2.35), Double.valueOf(3800.0));

    shooterAngleMap.put(Double.valueOf(1.27), Double.valueOf(1.0));
    shooterAngleMap.put(Double.valueOf(1.88), Double.valueOf(7.0));
    shooterAngleMap.put(Double.valueOf(2.35), Double.valueOf(10.0));

    // 1.27m  2600rpm 0deg
    // 1.88m  2850rpm 7deg
    // 3400 8.5
    // 2.35m  3800rpm 10deg
    // +200rpm + 3.2deg
  }

  @Override
  public void periodic() {
    
    NetworkTableInstance.getDefault().getTable("AutoCentering").getEntry("RotatonInRange").setBoolean(getCenteringRotationError() < 17);

    output0 = DIO_0.get();
    sensorValues[0] = output0;
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("DIO_0").setBoolean(output0);

    output1 = DIO_1.get();
    sensorValues[1] = output1;
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("DIO_1").setBoolean(output1);
    
    if (LimelightHelpers.getTV("limelight-april")) {
      robotDrive.visionPose(LimelightHelpers.getBotPose2d("limelight-april"), Timer.getFPGATimestamp());
    }

    pose = robotDrive.getPose();
    angle = pose.getRotation().getDegrees();

    if (DriverStation.isFMSAttached()) {
      if (DriverStation.getAlliance().isPresent()) {
        red = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
      }
    }
    else if (pose.getX() > 0) red = true;
    else red = false;

    if (red) {
      if (angle > 0) angle -= 180;
      else angle += 180;
      angle *= -1;
    }

    NetworkTableInstance.getDefault().getTable("Centering").getEntry("Red: ").setBoolean(red);

    speaker = Constants.Field.BLUE_SPEAKER_FROM_CENTER;
    xDistance = Math.abs(pose.getX()) - Math.abs(speaker.getX());
    yDistance = Math.abs(pose.getY()) - Math.abs(speaker.getY());

    xDistance *= -1;

    distanceToSpeaker = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
    angleToSpeaker = Math.atan2(yDistance, xDistance) * (180/Math.PI);

    speedMap = shooterSpeedMap.get(distanceToSpeaker) + 2000;
    angleMap = shooterAngleMap.get(Double.valueOf(distanceToSpeaker));

    NetworkTableInstance.getDefault().getTable("Centering").getEntry("xDistance").setDouble(xDistance);
    NetworkTableInstance.getDefault().getTable("Centering").getEntry("yDistance").setDouble(yDistance);
    NetworkTableInstance.getDefault().getTable("Centering").getEntry("DistanceToSpeaker").setDouble(distanceToSpeaker);
    NetworkTableInstance.getDefault().getTable("Centering").getEntry("Angle").setDouble(angle);
    NetworkTableInstance.getDefault().getTable("Centering").getEntry("AngleToSpeaker").setDouble(angleToSpeaker);
    NetworkTableInstance.getDefault().getTable("Centering").getEntry("AngleDifference").setDouble(angle - angleToSpeaker);

    NetworkTableInstance.getDefault().getTable("Centering").getEntry("getShooterSpeedDistance").setDouble(distanceToSpeaker);
    NetworkTableInstance.getDefault().getTable("Centering").getEntry("ShooterSpeedInterpolation").setDouble(shooterAngleMap.get(Double.valueOf(distanceToSpeaker)));

    NetworkTableInstance.getDefault().getTable("Centering").getEntry("getShooterAngleDistance").setDouble(distanceToSpeaker);
    NetworkTableInstance.getDefault().getTable("Centering").getEntry("getShooterAngleInterpolation").setDouble(shooterAngleMap.get(Double.valueOf(distanceToSpeaker)));
    NetworkTableInstance.getDefault().getTable("Centering").getEntry("getShooterAngleInterpolationPlus").setDouble(shooterAngleMap.get(Double.valueOf(distanceToSpeaker)) + 3.2);

    NetworkTableInstance.getDefault().getTable("Centering").getEntry("speedMap").setDouble(speedMap);
    NetworkTableInstance.getDefault().getTable("Centering").getEntry("angleMap").setDouble(angleMap);
  }

  public boolean getIntakeSensor(int sensorNum) {

    // DIO 0 inverted

    if (sensorNum > -1 && sensorNum < sensorValues.length) 
      return sensorValues[sensorNum];

    return false;
  }

  public double SpeakerRotation() {

    double rotation = (angle - angleToSpeaker) * (0.025);
    if (red) rotation *= -1;

    if (rotation > 0.35) rotation = 0.35;
    if (rotation < -0.35) rotation = -0.35;


    NetworkTableInstance.getDefault().getTable("Centering").getEntry("Rotation").setDouble(rotation);

    return rotation;
  }

  public boolean onSide() {
    
    if (red && pose.getX() > 1) return true;
    else if (!red && pose.getX() < -1) return true;
    return false;
  }

  public boolean inRange() {
    return distanceToSpeaker < 2.5;
  }

  public double getCenteringRotationError() {
    return Math.abs(angle - angleToSpeaker);
  }
}
