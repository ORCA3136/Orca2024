// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
//import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunShooterCommand;
import frc.robot.commands.RunArmCommand;
import frc.robot.commands.SetSwerveXCommand;
import frc.robot.commands.FollowPathCommand;
import frc.robot.commands.PathfindThenFollowPathCommand;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.NoteOffIntake;
import frc.robot.commands.ShootSpeaker;
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

import edu.wpi.first.wpilibj.DataLogManager;

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

  /*
  // Configure trajectory stuff
    // For forwards
  TrajectoryConfig config = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics)
      .setReversed(false);

    // For backwards
  TrajectoryConfig configReversed = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics)
      .setReversed(true);

  // Trajectories
  Trajectory driveStraightForward = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(0.5, 0)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(1, 0, new Rotation2d(0)),
      config);

  Trajectory driveStraightBackward = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(-0.5, 0)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(-1, 0, new Rotation2d(0)),
      configReversed);
      */

  // Sequential Shoot Commands
  ParallelCommandGroup shootNote = new ParallelCommandGroup(RunShooterCommand(1), new SequentialCommandGroup(new WaitCommand(3), RunIntakeCommand(0.5)));

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandJoystick m_secondaryController = new CommandJoystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
    
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
    autoChooser.addOption("First Auto", AutoBuilder.buildAuto("First Auto"));
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
    new JoystickButton(m_driverController, 1).whileTrue(RunIntakeCommand(0.6));
    new JoystickButton(m_driverController, 2).whileTrue(RunIntakeCommand(-0.3));
    new JoystickButton(m_driverController, 3).whileTrue(RunArmCommand(0.4));
    new JoystickButton(m_driverController, 4).onTrue(RunArmCommand(-0.25)).onFalse(RunArmCommand(0));
    new JoystickButton(m_driverController, 5).toggleOnTrue(m_ShooterSubsystem.reverseShootNote());
    new JoystickButton(m_driverController, 6).toggleOnTrue(m_ShooterSubsystem.shootNote());
    new JoystickButton(m_driverController, 7).whileTrue(RunShooterCommand(1));
    new JoystickButton(m_driverController, 8).whileTrue(ZeroHeading());
    
    //PID buttons
    //new JoystickButton(m_driverController, 5).onTrue(new ArmPID(100 * 0.04, m_ArmSubsystem));
    //new JoystickButton(m_driverController, 6).onTrue(new ArmPID(100 * 0.50, m_ArmSubsystem));
    //////new JoystickButton(m_driverController, 7).onTrue(new ArmPID(m_ArmSubsystem));
    //new JoystickButton(m_driverController, 8).onTrue(new ArmPID(100 * 0.30, m_ArmSubsystem));
    //m_secondaryController.button(5).onTrue(new ArmPID(m_ArmSubsystem));

    //0.52 Amp --- Intake level 0.3 --- Intake low 0.15 --- 0.01 Floor
//26.09
    //Pathfinding buttons
    //new JoystickButton(m_driverController, 6).whileTrue(GenerateTrajectoryCommand(driveStraightForward));
    //new JoystickButton(m_driverController, ).whileTrue(PathfindThenFollowPathCommand("ApproachAmp"));
    //m_secondaryController.button(6).onTrue(new ArmPID(m_ArmSubsystem));
    //m_secondaryController.button(7).onTrue(new ArmPID(m_ArmSubsystem));

    m_secondaryController.button(1).toggleOnTrue(m_robotDrive.toggleSpeakerCentering(m_driverController, m_SensorSubsystem));

    m_secondaryController.button(3).onTrue(new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem));
    m_secondaryController.button(4).onTrue(new ShootSpeaker(m_ShooterSubsystem, m_IntakeSubsystem));

    m_secondaryController.button(5).whileTrue(SetSwerveXCommand());
    m_secondaryController.button(7).onTrue(m_ClimberSubsystem.RunClimber(0.1)).onFalse(m_ClimberSubsystem.RunClimber(0));
    m_secondaryController.button(8).onTrue(m_ClimberSubsystem.RunClimber(-0.1)).onFalse(m_ClimberSubsystem.RunClimber(0));

    m_secondaryController.button(9).onTrue(m_ArmSubsystem.RunArmPID(0.1));
    m_secondaryController.button(10).onTrue(m_ArmSubsystem.RunArmPID(0.15));
    m_secondaryController.button(11).onTrue(m_ArmSubsystem.RunArmPID(0.2));
    m_secondaryController.button(12).onTrue(m_ArmSubsystem.RunArmPID(0.25));
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

  public final void StartPID() {
    
  }

  public final ParallelCommandGroup getShootNote() {
    return shootNote;
  } 

  public final double getLeftTrigger() {
    return m_driverController.getLeftTriggerAxis();
  }


  //private commands
  private final RunIntakeCommand RunIntakeCommand(double speed) {

    return new RunIntakeCommand(speed, m_IntakeSubsystem);

  }

  private final RunShooterCommand RunShooterCommand(double speed) {

    return new RunShooterCommand(-speed, m_ShooterSubsystem);

  }

  private final RunArmCommand RunArmCommand(double speed) {

    return new RunArmCommand(speed, m_ArmSubsystem);

  }

  private final SetSwerveXCommand SetSwerveXCommand() {

    return new SetSwerveXCommand(m_robotDrive);

  }

  private final FollowPathCommand FollowPathCommand(String pathName) {

    return new FollowPathCommand(m_robotDrive, pathName);

  }

  private final PathfindThenFollowPathCommand PathfindThenFollowPathCommand(String pathName) {

    return new PathfindThenFollowPathCommand(m_robotDrive, pathName);

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

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true, true));
  }

}
