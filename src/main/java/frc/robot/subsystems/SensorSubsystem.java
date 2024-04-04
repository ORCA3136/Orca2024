// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SensorSubsystem extends SubsystemBase {

  private static DigitalInput DIO_1;
  private static DigitalInput DIO_2;
  private boolean output1;
  private boolean output2;
  private boolean[] sensorValues;
  private DriveSubsystem robotDrive;

  private InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();

  ChassisSpeeds currentRobotSpeeds;
  ChassisSpeeds currentFieldSpeeds;
  double speed;
  double direction;
  double omegaSpeed;

  double xSpeed;
  double ySpeed;
  double tangentSpeed;
  double normalSpeed;

  boolean red;
  Pose2d pose;
  Translation2d speaker = Constants.Field.BLUE_SPEAKER_FROM_CENTER;
  double angle;
  
  double xDistance;
  double yDistance;

  double distanceToSpeaker;
  double angleToSpeaker;
  double radiansToSpeaker;

  public double speedMap;
  public double angleMap;
  public double verticalOffset;

  /** Creates a new SensorSubsystem. */
  public SensorSubsystem(DriveSubsystem drive) {

    LimelightHelpers.setPipelineIndex("limelight-note", 0);

    DIO_1 = new DigitalInput(1);
    DIO_2 = new DigitalInput(2);
    robotDrive = drive;

    sensorValues = new boolean[3];

    // More datapoints for 2, 2.5, 3, 3.5

    shooterSpeedMap.put(Double.valueOf(1.2), Double.valueOf(3300));
    shooterSpeedMap.put(Double.valueOf(1.5), Double.valueOf(3100));
    shooterSpeedMap.put(Double.valueOf(2), Double.valueOf(3000));
    shooterSpeedMap.put(Double.valueOf(2.5), Double.valueOf(3000));
    shooterSpeedMap.put(Double.valueOf(3), Double.valueOf(3000));
    shooterSpeedMap.put(Double.valueOf(3.5), Double.valueOf(3250));
    shooterSpeedMap.put(Double.valueOf(4), Double.valueOf(3600));
    shooterSpeedMap.put(Double.valueOf(4.5), Double.valueOf(4300));
    shooterSpeedMap.put(Double.valueOf(5.3), Double.valueOf(5000));
    shooterSpeedMap.put(Double.valueOf(6), Double.valueOf(5000));

    shooterAngleMap.put(Double.valueOf(1.2), Double.valueOf(1.5));
    shooterAngleMap.put(Double.valueOf(1.5), Double.valueOf(5));
    shooterAngleMap.put(Double.valueOf(2), Double.valueOf(8));
    shooterAngleMap.put(Double.valueOf(2.5), Double.valueOf(14.2));
    shooterAngleMap.put(Double.valueOf(3), Double.valueOf(17.5));
    shooterAngleMap.put(Double.valueOf(3.5), Double.valueOf(22.5));
    shooterAngleMap.put(Double.valueOf(4), Double.valueOf(26));
    shooterAngleMap.put(Double.valueOf(4.5), Double.valueOf(28));
    shooterAngleMap.put(Double.valueOf(5.3), Double.valueOf(28.5));
    shooterAngleMap.put(Double.valueOf(6), Double.valueOf(29));

  }

  @Override
  public void periodic() {
    
    NetworkTableInstance.getDefault().getTable("AutoCentering").getEntry("RotatonInRange").setBoolean(getCenteringRotationError() < 17);

    sensorValues[0] = false;

    output1 = !DIO_1.get();
    sensorValues[1] = output1;
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("DIO_1").setBoolean(output1);

    output2 = DIO_2.get();
    sensorValues[2] = output2;
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("DIO_2").setBoolean(output2);
    
    if (LimelightHelpers.getTV("limelight-april")) {
      if (DriverStation.isAutonomous())
        if (LimelightHelpers.getTA("limelight-april") > 0.25)
          robotDrive.visionPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight-april"), Timer.getFPGATimestamp());
      if (LimelightHelpers.getTA("limelight-april") > 0.125)
        robotDrive.visionPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight-april"), Timer.getFPGATimestamp());
    }

    pose = robotDrive.getPose();
    angle = pose.getRotation().getDegrees();

    if (DriverStation.isFMSAttached()) {
      if (DriverStation.getAlliance().isPresent()) {
        red = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
      }
    }
    else if (pose.getX() > 8.27) red = true;
    else red = false;

    if (red) {
      speaker = Constants.Field.RED_SPEAKER;
      xDistance = Math.abs(speaker.getX() - pose.getX());
      yDistance = speaker.getY() - pose.getY();

      if (angle > 0) angle -= 180;
      else angle += 180;
      // angle *= -1;
    }
    else {
      speaker = Constants.Field.BLUE_SPEAKER;
      xDistance = Math.abs(speaker.getX() - pose.getX());
      yDistance = pose.getY() - speaker.getY();
    }

    distanceToSpeaker = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
    angleToSpeaker = (Math.atan2(yDistance, xDistance) * (180/Math.PI));  // Angular offset from test values - 2
    if (red) angleToSpeaker -= 1;
    else angleToSpeaker += 2;
    radiansToSpeaker = Math.atan2(yDistance, xDistance) + 0.035;

    speedMap = shooterSpeedMap.get(distanceToSpeaker);
    angleMap = shooterAngleMap.get(Double.valueOf(distanceToSpeaker));


    currentRobotSpeeds = robotDrive.getRobotRelativeSpeeds();
    currentFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(currentRobotSpeeds, new Rotation2d(angle));
    speed = currentRobotSpeeds.vxMetersPerSecond;
    direction = currentRobotSpeeds.vyMetersPerSecond; // Direction
    omegaSpeed = currentRobotSpeeds.omegaRadiansPerSecond;

    xSpeed = speed * Math.sin(direction);
    ySpeed = speed * Math.cos(direction);
    tangentSpeed = xSpeed * Math.cos(angleToSpeaker * (Math.PI/180)) + ySpeed * Math.sin(angleToSpeaker * (Math.PI/180));
    normalSpeed = xSpeed * Math.sin(angleToSpeaker * (Math.PI/180)) + ySpeed * Math.cos(angleToSpeaker * (Math.PI/180));

    verticalOffset = normalSpeed * 7 /* * dist? */ ;
    // verticalOffset = 0;

    NetworkTableInstance.getDefault().getTable("Rotation").getEntry("Speed").setDouble(speed);
    NetworkTableInstance.getDefault().getTable("Rotation").getEntry("Direction").setDouble(direction);
    NetworkTableInstance.getDefault().getTable("Rotation").getEntry("OmegaSpeed").setDouble(omegaSpeed);

    NetworkTableInstance.getDefault().getTable("Rotation").getEntry("XSpeed").setDouble(xSpeed);
    NetworkTableInstance.getDefault().getTable("Rotation").getEntry("YSpeed").setDouble(ySpeed);
    NetworkTableInstance.getDefault().getTable("Rotation").getEntry("TangentSpeed").setDouble(tangentSpeed);
    NetworkTableInstance.getDefault().getTable("Rotation").getEntry("NormalSpeed").setDouble(normalSpeed);



    NetworkTableInstance.getDefault().getTable("Centering").getEntry("Red: ").setBoolean(red);

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

  public double SpeakerRotation(DriveSubsystem m_DriveSubsystem) {

    double centeringOffset = tangentSpeed * 40;

    double rotationDifference = (angle - (angleToSpeaker + centeringOffset));

    double rotation = 0.0;

    if (rotationDifference > 25) rotation = 0.3;
    else if (rotationDifference < -25) rotation = -0.3;
    else if (rotationDifference > 0) rotation = rotationDifference * 0.0125 + 0.015;
    else if (rotationDifference < 0) rotation = rotationDifference * 0.0125 - 0.015;

    // 0 - 10 degrees offset



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

  public double getCenteringAngle() {
    return angleMap;
  }

  public double getCenteringSpeed() {
    return speedMap;
  }
}
