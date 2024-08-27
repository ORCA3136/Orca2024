// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.choreo.lib.Choreo;
//import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
import frc.robot.commands.ForwardClimb;
import frc.robot.commands.NOTNOTNoteSuck;
import frc.robot.commands.NoteOffFlywheel;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.SetSwerveXCommand;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.NoteOffIntake;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.SpeakerCentering;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.AutoBackupShoot;
import frc.robot.commands.AutoSpeakerCentering;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Trajectories;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandJoystick m_secondaryController = new CommandJoystick(1);

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final SensorSubsystem m_SensorSubsystem = new SensorSubsystem(m_robotDrive);

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(m_SensorSubsystem);
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem(this);
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final Trajectories m_trajectories = new Trajectories(m_robotDrive);
  private final SendableChooser<Command> autoChooser;
  private final Field2d field;

  private PIDController AutoDrivePID;
  private PIDController AutoTurnPID;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
     //for pathplanner logging
     
     AutoDrivePID = new PIDController(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD);
     AutoTurnPID = new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD);
     
     field = new Field2d();
     SmartDashboard.putData("Field", field);
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

  
    m_ArmSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_ArmSubsystem.AutomaticPositioning(),
        m_ArmSubsystem));
    
    m_IntakeSubsystem.setDefaultCommand(
      new NoteOffIntake(m_IntakeSubsystem, m_SensorSubsystem));

    Trajectory[] blueTrajectories = m_trajectories.GetTrajectories(false);
    Trajectory[] redTrajectories = m_trajectories.GetTrajectories(true);
    
    autoChooser = new SendableChooser<>(); // Default auto will be `Commands.none()`

    autoChooser.addOption("Blue - Amp then centerline", new SequentialCommandGroup(
      new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(90)),  
      // GenerateTrajectoryCommand(blueTrajectories[1]),
      m_robotDrive.GenerateChoreoPath("ScoreAmp", false),
      
      m_IntakeSubsystem.RunIntakeCommand(0.3),
      m_ShooterSubsystem.shootNote(700),

      Commands.waitSeconds(0.5),

      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ShooterSubsystem.shootNote(0),
      new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(25)),

      new ParallelCommandGroup(m_robotDrive.GenerateChoreoPath("AmpWall", false),
        new SequentialCommandGroup(Commands.waitSeconds(0.8), m_IntakeSubsystem.RunIntakeCommand(0.5), new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(3)))),

      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(1),

      new ParallelCommandGroup(m_robotDrive.GenerateChoreoPath("AmpNoteShoot", false),
        new SequentialCommandGroup(m_IntakeSubsystem.RunIntakeCommand(0.5), Commands.waitSeconds(0.3), new NoteOffFlywheel(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5))),

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      new ParallelCommandGroup(m_robotDrive.GenerateChoreoPath("AmpSecNote", false),
        new SequentialCommandGroup(Commands.waitSeconds(0.4), m_IntakeSubsystem.RunIntakeCommand(0.5), new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(3)))),

      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(0.5),

      new ParallelCommandGroup(m_robotDrive.GenerateChoreoPath("AmpSecNoteShoot", false),
        new SequentialCommandGroup(m_IntakeSubsystem.RunIntakeCommand(0.5), Commands.waitSeconds(0.2), new NoteOffFlywheel(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5))),

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      m_robotDrive.GenerateChoreoPath("AmpThirdNote", false)

    ));

    autoChooser.addOption("Blue - Speaker then source side centerline", new SequentialCommandGroup(
      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)).withTimeout(3),
      new AutoBackupShoot(m_ShooterSubsystem, m_SensorSubsystem, m_IntakeSubsystem),

      new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(25)),
    
      new ParallelCommandGroup(m_robotDrive.GenerateChoreoPath("SourceCenterLine", false),
        new SequentialCommandGroup(Commands.waitSeconds(1), m_IntakeSubsystem.RunIntakeCommand(0.5), new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(3)))),

      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(0.75),

      new ParallelCommandGroup(new SequentialCommandGroup(m_IntakeSubsystem.RunIntakeCommand(0.5), Commands.waitSeconds(0.3), new NoteOffFlywheel(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5)),
        m_robotDrive.GenerateChoreoPath("SourceSecNoteShoot", false)),

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      new ParallelCommandGroup(m_robotDrive.GenerateChoreoPath("SourceSecNote", false),
        new SequentialCommandGroup(Commands.waitSeconds(0.6), m_IntakeSubsystem.RunIntakeCommand(0.5), new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(3)))),
      
      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(0.75),

      new ParallelCommandGroup(new SequentialCommandGroup(m_IntakeSubsystem.RunIntakeCommand(0.5), Commands.waitSeconds(0.3), new NoteOffFlywheel(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5)),
        m_robotDrive.GenerateChoreoPath("SourceNoteShoot", false)),

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      m_robotDrive.GenerateChoreoPath("SourceThirdNote", false)
      
    ));

    autoChooser.addOption("Blue - 4 Note", new SequentialCommandGroup(

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      new ParallelRaceGroup(m_robotDrive.GenerateChoreoPath("StageNote", false),
        new SequentialCommandGroup(m_IntakeSubsystem.RunIntakeCommand(0.5), new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(3)),
        Commands.waitSeconds(1.4), new NoteOffFlywheel(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(2))),

      Commands.waitSeconds(0.1),

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      new ParallelRaceGroup(m_robotDrive.GenerateChoreoPath("MiddleNote", false),
        new SequentialCommandGroup(m_IntakeSubsystem.RunIntakeCommand(0.5), new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(3)),
        Commands.waitSeconds(0.9), new NoteOffFlywheel(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(2))),

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(3)),
      Commands.waitSeconds(0.3),

      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(0.4),
      m_IntakeSubsystem.RunIntakeCommand(0.5),
      Commands.waitSeconds(0.1),
      new NoteOffFlywheel(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      m_robotDrive.GenerateChoreoPath("4NoteCenter", false)

    ));

    autoChooser.addOption("Red - Amp then centerline", new SequentialCommandGroup(
      new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(90)),  
      // GenerateTrajectoryCommand(blueTrajectories[1]),
      m_robotDrive.GenerateChoreoPath("ScoreAmp", true),
      
      m_IntakeSubsystem.RunIntakeCommand(0.3),
      m_ShooterSubsystem.shootNote(700),

      Commands.waitSeconds(0.5),

      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ShooterSubsystem.shootNote(0),
      new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(25)),

      new ParallelCommandGroup(m_robotDrive.GenerateChoreoPath("AmpWall", true),
        new SequentialCommandGroup(Commands.waitSeconds(0.8), m_IntakeSubsystem.RunIntakeCommand(0.5), new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(3)))),

      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(1),

      new ParallelCommandGroup(m_robotDrive.GenerateChoreoPath("AmpNoteShoot", true),
        new SequentialCommandGroup(m_IntakeSubsystem.RunIntakeCommand(0.5), Commands.waitSeconds(0.3), new NoteOffFlywheel(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5))),

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      new ParallelCommandGroup(m_robotDrive.GenerateChoreoPath("AmpSecNote", true),
        new SequentialCommandGroup(Commands.waitSeconds(0.4), m_IntakeSubsystem.RunIntakeCommand(0.5), new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(3)))),

      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(0.5),

      new ParallelCommandGroup(m_robotDrive.GenerateChoreoPath("AmpSecNoteShoot", true),
        new SequentialCommandGroup(m_IntakeSubsystem.RunIntakeCommand(0.5), Commands.waitSeconds(0.2), new NoteOffFlywheel(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5))),

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      m_robotDrive.GenerateChoreoPath("AmpThirdNote", true)

    ));

    autoChooser.addOption("Red - Speaker then source side centerline", new SequentialCommandGroup(
      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)).withTimeout(3),
      new AutoBackupShoot(m_ShooterSubsystem, m_SensorSubsystem, m_IntakeSubsystem),

      new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(25)),
    
      new ParallelCommandGroup(m_robotDrive.GenerateChoreoPath("SourceCenterLine", true),
        new SequentialCommandGroup(Commands.waitSeconds(1), m_IntakeSubsystem.RunIntakeCommand(0.5), new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(3)))),

      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(0.75),

      new ParallelCommandGroup(new NoteOffFlywheel(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),
        m_robotDrive.GenerateChoreoPath("SourceSecNoteShoot", true)),

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      new ParallelCommandGroup(m_robotDrive.GenerateChoreoPath("SourceSecNote", true),
        new SequentialCommandGroup(Commands.waitSeconds(0.6), m_IntakeSubsystem.RunIntakeCommand(0.5), new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(3)))),
      
      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(0.75),

      new ParallelCommandGroup(new NoteOffFlywheel(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),
        m_robotDrive.GenerateChoreoPath("SourceNoteShoot", true)),

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      m_robotDrive.GenerateChoreoPath("SourceThirdNote", true)
      
    ));

    autoChooser.addOption("Red - 4 Note", new SequentialCommandGroup(

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      new ParallelRaceGroup(m_robotDrive.GenerateChoreoPath("StageNote", true),
        new SequentialCommandGroup(m_IntakeSubsystem.RunIntakeCommand(0.5), new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(3)),
        Commands.waitSeconds(1.4), new NoteOffFlywheel(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(2))),

      Commands.waitSeconds(0.1),

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      new ParallelRaceGroup(m_robotDrive.GenerateChoreoPath("MiddleNote", true),
        new SequentialCommandGroup(m_IntakeSubsystem.RunIntakeCommand(0.5), new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(3)),
        Commands.waitSeconds(0.9), new NoteOffFlywheel(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(2))),

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(3)),
      Commands.waitSeconds(0.3),

      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(0.4),
      m_IntakeSubsystem.RunIntakeCommand(0.5),
      Commands.waitSeconds(0.1),
      new NoteOffFlywheel(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),

      new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
        new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem)),

      m_robotDrive.GenerateChoreoPath("4NoteCenter", true)

    ));

    autoChooser.addOption("Shoot stay still", new SequentialCommandGroup(

    new ParallelRaceGroup(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem),
      new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem))
      
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
    
    new JoystickButton(m_driverController, 1).whileTrue(new RunIntakeCommand(0.5, m_IntakeSubsystem));
    new JoystickButton(m_driverController, 2).onTrue(new ParallelCommandGroup(new RunIntakeCommand(-0.3, m_IntakeSubsystem), m_ShooterSubsystem.shootNote(Constants.ShooterConstants.reverse))).onFalse(m_ShooterSubsystem.shootNote(0));
    new JoystickButton(m_driverController, 3).whileTrue(new RunCommand(() -> m_ArmSubsystem.ManualPositioning(0.3), m_ArmSubsystem));
    new JoystickButton(m_driverController, 4).whileTrue(new RunCommand(() -> m_ArmSubsystem.ManualPositioning(-0.2), m_ArmSubsystem));

    new JoystickButton(m_driverController, 5).onTrue(m_ShooterSubsystem.shootNote(Constants.ShooterConstants.reverse)).onFalse(m_ShooterSubsystem.shootNote(0));
    new JoystickButton(m_driverController, 6).onTrue(m_ShooterSubsystem.shootNote(3500)).onFalse(m_ShooterSubsystem.shootNote(0));
    new JoystickButton(m_driverController, 7).onTrue(m_ClimberSubsystem.RunClimber(1)).onFalse(m_ClimberSubsystem.RunClimber(0));
    new JoystickButton(m_driverController, 8).onTrue(m_ClimberSubsystem.RunClimber(-1)).onFalse(m_ClimberSubsystem.RunClimber(0));
    
    // new JoystickButton(m_driverController, 9).onTrue();
    new JoystickButton(m_driverController, 10).whileTrue(new ZeroHeading(m_robotDrive));

    //---------------------------------------------------------------------------------------------------------------------------------

    // m_secondaryController.button(1).whileTrue(new TurnToAngle(m_robotDrive, m_SensorSubsystem, 90));
    // m_secondaryController.button(2).whileTrue(new TurnToAngle(m_robotDrive, m_SensorSubsystem, -90));
    // m_secondaryController.button(3).onTrue(new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(6)));
    // m_secondaryController.button(4).onTrue();

    m_secondaryController.button(5).onTrue(new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(12)));
    m_secondaryController.button(6).onTrue(new NoteOffFlywheel(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(1.5));
    m_secondaryController.button(7).onTrue(m_ClimberSubsystem.ResetEncoders());
    m_secondaryController.button(8).whileTrue(new ForwardClimb(m_ArmSubsystem, m_ClimberSubsystem));

    m_secondaryController.button(9).onTrue(new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(2)));
    m_secondaryController.button(10).onTrue(new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(25)));
    m_secondaryController.button(11).onTrue(new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(70)));
    m_secondaryController.button(12).onTrue(new InstantCommand(() -> m_ArmSubsystem.setTrapezoidalSetpoint(90)));



    BooleanSupplier LeftTriggerSupplier = new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        if (m_driverController.getLeftTriggerAxis() > 0.5) return true;
        else return false;
      }
    };
    Trigger LeftTrigger = new Trigger(LeftTriggerSupplier);

    LeftTrigger.whileTrue(new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem));
    // LeftTrigger.whileTrue(new AutoSpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_IntakeSubsystem));

    BooleanSupplier RightTriggerSupplier = new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        if (m_driverController.getRightTriggerAxis() > 0.5) return true;
        else return false;
      }
    };
    Trigger RightTrigger = new Trigger(RightTriggerSupplier);

    // Change to shoot routine
    RightTrigger.whileTrue(new SpeakerCentering(m_ShooterSubsystem, m_SensorSubsystem, m_ArmSubsystem, m_robotDrive, m_IntakeSubsystem, m_driverController));
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
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true, true));
  }
}


/**
 * How well does coral identify notes? - Picks up farther notes over closer notes
 * Do trajectories work well with timeouts?
 * 
 * Find a wait command for commands
 */