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
  private boolean[] sensorValues;
  private DriveSubsystem robotDrive;

  private InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();

  // True means note is in correct position
  private boolean IntakeState = false;
  


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
    DIO_0 = new DigitalInput(0);
    robotDrive = drive;

    sensorValues = new boolean[1];

    shooterSpeedMap.put(Double.valueOf(1.27), Double.valueOf(2600.0));
    shooterSpeedMap.put(Double.valueOf(1.88), Double.valueOf(2850.0));
    shooterSpeedMap.put(Double.valueOf(2.35), Double.valueOf(3800.0));

    shooterAngleMap.put(Double.valueOf(1.27), Double.valueOf(1.0));
    shooterAngleMap.put(Double.valueOf(1.88), Double.valueOf(7.0));
    shooterAngleMap.put(Double.valueOf(2.35), Double.valueOf(10.0));

    DataLogManager.log("ShooterAngleMap: "+shooterAngleMap.get(Double.valueOf(2)));
    DataLogManager.log("ShooterSpeedMap: "+shooterSpeedMap.get(Double.valueOf(2)));
    


    // 1.27m  2600rpm 0deg
    // 1.88m  2850rpm 7deg
    // 3400 8.5
    // 2.35m  3800rpm 10deg
    // +200rpm + 3.2deg
  }

  @Override
  public void periodic() {
    
    output0 = DIO_0.get();
    sensorValues[0] = output0;
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("DIO_0").setBoolean(output0);
    
    if (LimelightHelpers.getTV("limelight")) {
      robotDrive.visionPose(LimelightHelpers.getBotPose2d("limelight"), Timer.getFPGATimestamp());
    }

    pose = robotDrive.getPose();
    angle = pose.getRotation().getDegrees();
    speaker = Constants.Field.BLUE_SPEAKER_FROM_CENTER;
    xDistance = pose.getX() - speaker.getX();
    yDistance = pose.getY() - speaker.getY();
    distanceToSpeaker = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
    angleToSpeaker = Math.atan2(yDistance, xDistance) * (180/Math.PI);

    speedMap = shooterSpeedMap.get(distanceToSpeaker) + 200;
    angleMap = shooterAngleMap.get(Double.valueOf(distanceToSpeaker)) + 3.2;

    NetworkTableInstance.getDefault().getTable("Centering").getEntry("xDistance").setDouble(xDistance);
    NetworkTableInstance.getDefault().getTable("Centering").getEntry("yDistance").setDouble(yDistance);
    NetworkTableInstance.getDefault().getTable("Centering").getEntry("DistanceToSpeaker").setDouble(distanceToSpeaker);
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

  public void changeIntakeState() {

  }

  public boolean getIntakeSensor(int sensorNum) {

    if (sensorNum > -1 && sensorNum < sensorValues.length) 
      return sensorValues[sensorNum];

    return false;
  }

  public double StartSpeakerRotation(SensorSubsystem sensor, ArmSubsystem arm, ShooterSubsystem shooter) {

    double rotation = (angle - angleToSpeaker) * (0.02);

    if (rotation > 0.4) rotation = 0.4;
    if (rotation < -0.4) rotation = -0.4;

    NetworkTableInstance.getDefault().getTable("Centering").getEntry("Rotation").setDouble(rotation);

    arm.SetPIDNOTNOTSensor(sensor);
    shooter.shootNoteNOTNOTSensor(sensor);

    return rotation;
  }
}
