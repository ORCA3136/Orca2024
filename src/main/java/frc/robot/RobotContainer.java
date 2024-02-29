// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.choreo.lib.Choreo;
//import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.NOTNOTNoteSuck;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunArmCommand;
import frc.robot.commands.SetSwerveXCommand;
import frc.robot.commands.FollowPathCommand;
import frc.robot.commands.PathfindThenFollowPathCommand;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.NoteOffIntake;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.VerticalCentering;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.RobotController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem(this);
  private final SensorSubsystem m_SensorSubsystem = new SensorSubsystem(m_robotDrive);
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final SendableChooser<Command> autoChooser;
  private final Field2d field;


  
  // Configure trajectory stuff
    // For forwards
  TrajectoryConfig config = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics)
      .setReversed(false);

  // Trajectories
  Trajectory driveForward = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(1.45, 5.4, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(2, 5.5)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(2.6, 5.4, new Rotation2d(0)),
      config);

  Trajectory driveBackward = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(2.6, 5.4, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(2, 5.3)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(1.45, 5.4, new Rotation2d(0)),
      config);

  Trajectory driveFartherForward = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(1.45, 5.4, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(2.4, 5.5)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3.6, 5.4, new Rotation2d(0)),
      config);

  Trajectory driveToAmp = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(1.45, 7.0, new Rotation2d(-Math.PI/2)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(1.7, 7.5)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(1.75, 7.7, new Rotation2d(-Math.PI/2)),
      config);

  Trajectory driveToAmpNote = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(1.75, 7.7, new Rotation2d(-Math.PI/2)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(2.0, 7.5)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(2.7, 7.05, new Rotation2d(0)),
      config);

  Trajectory driveToAmpFromNote = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(2.5, 7.25, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(2.0, 7.5)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(1.75, 7.7, new Rotation2d(-Math.PI/2)),
      config);

  Trajectory driveAcrossLine = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(1.75, 7.7, new Rotation2d(-Math.PI/2)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(2.2, 7.4)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(7, 6.5, new Rotation2d(0)),
      config);
      
  Command forwardTrajectory;
  Command forwardTrajectory2;
  Command backwardTrajectory;
  Command ampTrajectory;
  Command ampNoteTrajectory;
  Command noteAmpTrajectory;
  Command ampDriveTrajectory;

  Command autoSpeakerCentering;

  private PIDController AutoDrivePID;
  private PIDController AutoTurnPID;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandJoystick m_secondaryController = new CommandJoystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
     //for pathplanner logging
     
     AutoDrivePID = new PIDController(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD);
     AutoTurnPID = new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD);

     forwardTrajectory = GenerateTrajectoryCommand(driveForward);
     forwardTrajectory2 = GenerateTrajectoryCommand(driveFartherForward);
     backwardTrajectory = GenerateTrajectoryCommand(driveBackward);
     ampTrajectory = GenerateTrajectoryCommand(driveToAmp);
     ampNoteTrajectory = GenerateTrajectoryCommand(driveToAmpNote);
     noteAmpTrajectory = GenerateTrajectoryCommand(driveToAmpFromNote);
     ampDriveTrajectory = GenerateTrajectoryCommand(driveAcrossLine);

     autoSpeakerCentering = m_robotDrive.autoSpeakerCentering(m_SensorSubsystem);
     
     field = new Field2d();
     SmartDashboard.putData("Field", field);
     PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            DataLogManager.log("CurrentPose"+pose);
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            DataLogManager.log("Target Pose"+pose);
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            DataLogManager.log("Path"+poses);
            field.getObject("path").setPoses(poses);
        });
    //end pathplanner additional logging

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    autoChooser = new SendableChooser<>(); // Default auto will be `Commands.none()`
    autoChooser.addOption("Shoot then drive", new SequentialCommandGroup(
      
      m_ArmSubsystem.SetPIDPosition(35),
      m_ShooterSubsystem.shootNote(4800),
      Commands.waitSeconds(0.5),

      m_ArmSubsystem.SetPIDPosition(2.5),
      Commands.waitSeconds(2),


      m_IntakeSubsystem.RunIntakeCommand(0.6),
      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(0),
      forwardTrajectory,
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(10),
      new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),

      m_ArmSubsystem.SetPIDPosition(2.5),
      backwardTrajectory,

      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(4800),
      Commands.waitSeconds(2),
      m_IntakeSubsystem.RunIntakeCommand(0.6),

      Commands.waitSeconds(0.5),
      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0),
      forwardTrajectory2

    ));    
    autoChooser.addOption("Amp score", new SequentialCommandGroup(
      m_ArmSubsystem.SetPIDPosition(95),  
      ampTrajectory,
      
      m_ShooterSubsystem.shootNote(700),

      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(0),
      m_ArmSubsystem.SetPIDPosition(45),

      Commands.waitSeconds(0.5),
      m_ArmSubsystem.SetPIDPosition(3),
      Commands.waitSeconds(1),
      m_IntakeSubsystem.RunIntakeCommand(0.3),

      ampNoteTrajectory,
      m_ArmSubsystem.SetPIDPosition(40),
      
      noteAmpTrajectory,
      m_ArmSubsystem.SetPIDPosition(95),
      Commands.waitSeconds(1),
      m_ShooterSubsystem.shootNote(700),
      Commands.waitSeconds(0.5),
      
      m_ShooterSubsystem.shootNote(0),
      m_ArmSubsystem.SetPIDPosition(60),
      m_IntakeSubsystem.RunIntakeCommand(0),
      Commands.waitSeconds(0.5),
      m_ArmSubsystem.SetPIDPosition(25),
      Commands.waitSeconds(1),

      ampDriveTrajectory
    )); 
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    //Main buttons
    new JoystickButton(m_driverController, 1).whileTrue(RunIntakeCommand(1));
    new JoystickButton(m_driverController, 2).whileTrue(RunIntakeCommand(-0.3));
    new JoystickButton(m_driverController, 3).onTrue(m_ArmSubsystem.RunArm(0.2)).onFalse(m_ArmSubsystem.RunArm(0));
    new JoystickButton(m_driverController, 4).onTrue(m_ArmSubsystem.RunArm(-0.2)).onFalse(m_ArmSubsystem.RunArm(0));
    new JoystickButton(m_driverController, 5).onTrue(m_ShooterSubsystem.shootNote(Constants.ShooterConstants.reverse)).onFalse(m_ShooterSubsystem.shootNote(0));
    new JoystickButton(m_driverController, 6).onTrue(m_ShooterSubsystem.shootNote(5000)).onFalse(m_ShooterSubsystem.shootNote(0));
    new JoystickButton(m_driverController, 7).onTrue(m_ClimberSubsystem.RunClimber(0.6)).onFalse(m_ClimberSubsystem.RunClimber(0));
    new JoystickButton(m_driverController, 8).onTrue(m_ClimberSubsystem.RunClimber(-0.6)).onFalse(m_ClimberSubsystem.RunClimber(0));
    new JoystickButton(m_driverController, 10).whileTrue(ZeroHeading());

    m_secondaryController.button(1).onTrue(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem)).onFalse(m_robotDrive.regularDrive(m_driverController));
    //m_secondaryController.button(2).onTrue();
    m_secondaryController.button(3).onTrue(new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(1.5));
    m_secondaryController.button(4).onTrue(new ShootSpeaker(m_ShooterSubsystem, m_IntakeSubsystem, 4000).withTimeout(3.5));

    m_secondaryController.button(5).onTrue(new ParallelCommandGroup(m_ShooterSubsystem.shootNoteNOTNOTSensor(m_SensorSubsystem), m_ArmSubsystem.SetPIDNOTNOTSensor(m_SensorSubsystem))).onFalse(m_ShooterSubsystem.shootNote(0));
    // m_secondaryController.button(6).onTrue(Commands.waitSeconds(0.5).andThen(RunIntakeCommand(0.3)));
    // m_secondaryController.button(7).onTrue(m_ArmSubsystem.SetSetpoint2());
    // m_secondaryController.button(8).onTrue();

    m_secondaryController.button(9).onTrue(m_ArmSubsystem.SetPIDPosition(3));
    m_secondaryController.button(10).onTrue(m_ArmSubsystem.SetPIDPosition(25));
    m_secondaryController.button(11).onTrue(m_ArmSubsystem.SetPIDPosition(70));
    m_secondaryController.button(12).onTrue(m_ArmSubsystem.SetPIDPosition(90));



    BooleanSupplier LeftTriggerSupplier = new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        if (m_driverController.getLeftTriggerAxis() > 0.5) return true;
        else return false;
      }
    };
    Trigger LeftTrigger = new Trigger(LeftTriggerSupplier);

    LeftTrigger.whileTrue(NOTNOTNoteSuck());
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return autoChooser.getSelected();
  }


  //public methods
  public final int getPOV() {
    return m_driverController.getPOV();
  }

  public final double getLeftTrigger() {
    return m_driverController.getLeftTriggerAxis();
  }


  //private commands
  private final RunIntakeCommand RunIntakeCommand(double speed) {

    return new RunIntakeCommand(speed, m_IntakeSubsystem);

  }

  private final NOTNOTNoteSuck NOTNOTNoteSuck() {

    return new NOTNOTNoteSuck(m_robotDrive);

  }

  private final ZeroHeading ZeroHeading() {

    return new ZeroHeading(m_robotDrive);

  }

  private Command GenerateTrajectoryCommand(Trajectory trajectory) {
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Run path following command, then stop at the end.
    return swerveControllerCommand;
  }

  // Trajectory command generator
  // public Command DriveTrajectory(String Trajectory) {

  //   Optional<Alliance> RobotAlliance;
  //   RobotAlliance = DriverStation.getAlliance();

  //   return Choreo.choreoSwerveCommand(
  //     Choreo.getTrajectory("DriveForward"), 
  //     () -> (m_robotDrive.getPose()), 
  //     AutoDrivePID, AutoDrivePID, AutoTurnPID, 
  //     (ChassisSpeeds speeds) -> m_robotDrive.driveRobotRelative(speeds),
  //     () -> RobotAlliance.get() == Alliance.Red, 
  //     m_robotDrive
  //     );

  // }
}