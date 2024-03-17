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
import frc.robot.commands.ForwardClimb;
import frc.robot.commands.NOTNOTNoteSuck;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunArmCommand;
import frc.robot.commands.SetSwerveXCommand;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.NoteOffIntake;
import frc.robot.commands.NoteOffIntake;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.SpeakerCentering;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Trajectories;
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
  private final SensorSubsystem m_SensorSubsystem = new SensorSubsystem(m_robotDrive);

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(m_SensorSubsystem);
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem(this);
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final Trajectories m_trajectories = new Trajectories(m_robotDrive);
  private final SendableChooser<Command> autoChooser;
  private final Field2d field;

      
  Command speakSourceNoteTrajectoryBlue, sourceNoteSpeakTrajectoryBlue;

  Command speakAmpNoteTrajectoryRed, ampNoteSpeakTrajectoryRed, speakSourceNoteTrajectoryRed, sourceNoteSpeakTrajectoryRed;

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

     autoSpeakerCentering = m_robotDrive.autoSpeakerCentering(m_SensorSubsystem);

     ArrayList<Trajectory> MidSpeakBlue = m_trajectories.getMiddleDoubleScore(false);
     ArrayList<Trajectory> DubAmpBlue = m_trajectories.getAmpDoubleScore(false);
     ArrayList<Trajectory> DriveOutBlue = m_trajectories.getDriveForward(false);
     ArrayList<Trajectory> TripSpeakBlue = m_trajectories.getTripleSpeakerScore(false);

     speakSourceNoteTrajectoryBlue = GenerateTrajectoryCommand(TripSpeakBlue.get(2));
     sourceNoteSpeakTrajectoryBlue = GenerateTrajectoryCommand(TripSpeakBlue.get(3));

     ArrayList<Trajectory> MidSpeakRed = m_trajectories.getMiddleDoubleScore(true);
     ArrayList<Trajectory> DubAmpRed = m_trajectories.getAmpDoubleScore(true);
     ArrayList<Trajectory> DriveOutRed = m_trajectories.getDriveForward(true);
     ArrayList<Trajectory> TripSpeakRed = m_trajectories.getTripleSpeakerScore(true);

     speakSourceNoteTrajectoryRed = GenerateTrajectoryCommand(TripSpeakRed.get(2));
     sourceNoteSpeakTrajectoryRed = GenerateTrajectoryCommand(TripSpeakRed.get(3));
     
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



    // Blue autos
    autoChooser.addOption("Blue - Triple speaker score", new SequentialCommandGroup(
      
      m_ShooterSubsystem.shootNote(2000),
      m_ArmSubsystem.SetPIDPosition(25),
      Commands.waitSeconds(0.25),

      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(0.5),

      m_ArmSubsystem.SetPIDPosition(1.5),
      Commands.waitSeconds(1.0),

      m_IntakeSubsystem.RunIntakeCommand(1),
      Commands.waitSeconds(0.3),

      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0.4),

      GenerateTrajectoryCommand(MidSpeakBlue.get(0)),
      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(0.25),

      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(10),
      new ParallelCommandGroup(new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),
        GenerateTrajectoryCommand(MidSpeakBlue.get(1))),
      m_ArmSubsystem.SetPIDPosition(2),

      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(1.25),
      m_IntakeSubsystem.RunIntakeCommand(1),

      Commands.waitSeconds(0.25),
      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0.5),

      GenerateTrajectoryCommand(TripSpeakBlue.get(0)),
      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(0.25),

      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(10),

      new ParallelCommandGroup(new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),
        GenerateTrajectoryCommand(TripSpeakBlue.get(1))),
      m_ArmSubsystem.SetPIDPosition(2),

      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(1.25),
      m_IntakeSubsystem.RunIntakeCommand(0.6),

      Commands.waitSeconds(0.25),
      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0)

    ));
    autoChooser.addOption("Blue - Double speaker score", new SequentialCommandGroup(
      
      m_ShooterSubsystem.shootNote(2000),
      m_ArmSubsystem.SetPIDPosition(1.5),
      Commands.waitSeconds(0.25),

      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(1.5),

      m_IntakeSubsystem.RunIntakeCommand(0.6),
      Commands.waitSeconds(0.25),

      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_IntakeSubsystem.RunIntakeCommand(0.4),

      GenerateTrajectoryCommand(MidSpeakBlue.get(0)),
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(10),
      new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),

      m_ArmSubsystem.SetPIDPosition(2),
      GenerateTrajectoryCommand(MidSpeakBlue.get(1)),

      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(2),
      m_IntakeSubsystem.RunIntakeCommand(0.6),

      Commands.waitSeconds(0.5),
      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0),
      GenerateTrajectoryCommand(MidSpeakBlue.get(2))

    ));
    autoChooser.addOption("Blue - Double amp score", new SequentialCommandGroup(
      m_ArmSubsystem.SetPIDPosition(90),  
      GenerateTrajectoryCommand(DubAmpBlue.get(0)),
      
      m_IntakeSubsystem.RunIntakeCommand(0.3),
      m_ShooterSubsystem.shootNote(700),

      Commands.waitSeconds(0.5),

      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ShooterSubsystem.shootNote(0),
      m_ArmSubsystem.SetPIDPosition(25),

      Commands.waitSeconds(0.5),
      m_ArmSubsystem.SetPIDPosition(3),
      Commands.waitSeconds(1),
      m_IntakeSubsystem.RunIntakeCommand(0.3),

      GenerateTrajectoryCommand(DubAmpBlue.get(1)),
      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(0.5),
      m_ArmSubsystem.SetPIDPosition(40),
      
      GenerateTrajectoryCommand(DubAmpBlue.get(2)),
      m_IntakeSubsystem.RunIntakeCommand(0.3),
      m_ArmSubsystem.SetPIDPosition(90),
      Commands.waitSeconds(1),
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ShooterSubsystem.shootNote(700),

      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(0),
      m_ArmSubsystem.SetPIDPosition(25),
      m_IntakeSubsystem.RunIntakeCommand(0),
      Commands.waitSeconds(0.25),

      GenerateTrajectoryCommand(DubAmpBlue.get(3))
    )); 
    autoChooser.addOption("Blue - Drive forward", new SequentialCommandGroup(
      
      m_ShooterSubsystem.shootNote(1000),
      Commands.waitSeconds(0.25),

      m_ArmSubsystem.SetPIDPosition(35),
      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(0.5),

      m_ArmSubsystem.SetPIDPosition(1.5),
      Commands.waitSeconds(1.0),

      m_IntakeSubsystem.RunIntakeCommand(0.6),
      Commands.waitSeconds(0.25),

      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(25),
    
      GenerateTrajectoryCommand(DriveOutBlue.get(0))
    ));



    // Red autos
    autoChooser.addOption("Red - Triple speaker score", new SequentialCommandGroup(
      
      m_ShooterSubsystem.shootNote(2000),
      m_ArmSubsystem.SetPIDPosition(25),
      Commands.waitSeconds(0.25),

      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(0.5),

      m_ArmSubsystem.SetPIDPosition(1.5),
      Commands.waitSeconds(1.0),

      m_IntakeSubsystem.RunIntakeCommand(1),
      Commands.waitSeconds(0.3),

      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0.4),

      GenerateTrajectoryCommand(MidSpeakRed.get(0)),
      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(0.25),

      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(10),
      new ParallelCommandGroup(new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),
        GenerateTrajectoryCommand(MidSpeakRed.get(1))),
      m_ArmSubsystem.SetPIDPosition(2),

      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(1.25),
      m_IntakeSubsystem.RunIntakeCommand(1),

      Commands.waitSeconds(0.25),
      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0.5),

      GenerateTrajectoryCommand(TripSpeakRed.get(0)),
      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(0.25),

      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(10),

      new ParallelCommandGroup(new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),
        GenerateTrajectoryCommand(TripSpeakRed.get(1))),
      m_ArmSubsystem.SetPIDPosition(2),

      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(1.25),
      m_IntakeSubsystem.RunIntakeCommand(0.6),

      Commands.waitSeconds(0.25),
      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0)

    ));
    autoChooser.addOption("Red - Double speaker score", new SequentialCommandGroup(
      
      m_ShooterSubsystem.shootNote(1000),
      Commands.waitSeconds(0.25),

      m_ArmSubsystem.SetPIDPosition(35),
      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(0.5),

      m_ArmSubsystem.SetPIDPosition(1.5),
      Commands.waitSeconds(1.5),

      m_IntakeSubsystem.RunIntakeCommand(0.6),
      Commands.waitSeconds(0.25),

      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_IntakeSubsystem.RunIntakeCommand(0.4),

      GenerateTrajectoryCommand(MidSpeakRed.get(0)),
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(10),
      new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),

      m_ArmSubsystem.SetPIDPosition(2),
      GenerateTrajectoryCommand(MidSpeakRed.get(1)),

      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(2),
      m_IntakeSubsystem.RunIntakeCommand(0.6),

      Commands.waitSeconds(0.5),
      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0),
      GenerateTrajectoryCommand(MidSpeakRed.get(2))

    ));
    autoChooser.addOption("Red - Double amp score", new SequentialCommandGroup(
      m_ArmSubsystem.SetPIDPosition(90),  
      GenerateTrajectoryCommand(DubAmpRed.get(0)),
      
      m_IntakeSubsystem.RunIntakeCommand(0.3),
      m_ShooterSubsystem.shootNote(700),

      Commands.waitSeconds(0.5),

      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ShooterSubsystem.shootNote(0),
      m_ArmSubsystem.SetPIDPosition(25),

      Commands.waitSeconds(0.5),
      m_ArmSubsystem.SetPIDPosition(3),
      Commands.waitSeconds(1),
      m_IntakeSubsystem.RunIntakeCommand(0.3),

      GenerateTrajectoryCommand(DubAmpRed.get(1)),
      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(0.5),
      m_ArmSubsystem.SetPIDPosition(40),
      
      GenerateTrajectoryCommand(DubAmpRed.get(2)),
      m_IntakeSubsystem.RunIntakeCommand(0.3),
      m_ArmSubsystem.SetPIDPosition(90),
      Commands.waitSeconds(1),
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ShooterSubsystem.shootNote(700),

      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(0),
      m_ArmSubsystem.SetPIDPosition(25),
      m_IntakeSubsystem.RunIntakeCommand(0),
      Commands.waitSeconds(0.25),

      GenerateTrajectoryCommand(DubAmpRed.get(3))
    )); 
    autoChooser.addOption("Red - Drive forward", new SequentialCommandGroup(
      
      m_ShooterSubsystem.shootNote(1000),
      Commands.waitSeconds(0.25),

      m_ArmSubsystem.SetPIDPosition(35),
      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(0.5),

      m_ArmSubsystem.SetPIDPosition(1.5),
      Commands.waitSeconds(1.0),

      m_IntakeSubsystem.RunIntakeCommand(0.6),
      Commands.waitSeconds(0.25),

      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(25),
    
      GenerateTrajectoryCommand(DriveOutRed.get(0))
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
    
    new JoystickButton(m_driverController, 1).whileTrue(new RunIntakeCommand(1, m_IntakeSubsystem)).onFalse(new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(1.5));
    new JoystickButton(m_driverController, 2).whileTrue(new ParallelCommandGroup(new RunIntakeCommand(-0.3, m_IntakeSubsystem), m_ShooterSubsystem.shootNote(Constants.ShooterConstants.reverse))).onFalse(m_ShooterSubsystem.shootNote(0));
    new JoystickButton(m_driverController, 3).onTrue(m_ArmSubsystem.RunArm(0.5)).onFalse(m_ArmSubsystem.RunArm(0));
    new JoystickButton(m_driverController, 4).onTrue(m_ArmSubsystem.RunArm(-0.3)).onFalse(m_ArmSubsystem.RunArm(0));

    new JoystickButton(m_driverController, 5).onTrue(m_ShooterSubsystem.shootNote(Constants.ShooterConstants.reverse)).onFalse(m_ShooterSubsystem.shootNote(0));
    new JoystickButton(m_driverController, 6).onTrue(m_ShooterSubsystem.shootNote(5500)).onFalse(m_ShooterSubsystem.shootNote(0));
    new JoystickButton(m_driverController, 7).onTrue(m_ClimberSubsystem.RunClimber(1)).onFalse(m_ClimberSubsystem.RunClimber(0));
    new JoystickButton(m_driverController, 8).onTrue(m_ClimberSubsystem.RunClimber(-1)).onFalse(m_ClimberSubsystem.RunClimber(0));
    
    // new JoystickButton(m_driverController, 9).onTrue();
    new JoystickButton(m_driverController, 10).whileTrue(new ZeroHeading(m_robotDrive));

    //---------------------------------------------------------------------------------------------------------------------------------

    // m_secondaryController.button(1).onTrue(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem)).onFalse(m_robotDrive.regularDrive(m_driverController));
    // m_secondaryController.button(2).onTrue(m_ArmSubsystem.SetPIDSensor(m_SensorSubsystem));
    // m_secondaryController.button(3).onTrue();
    // m_secondaryController.button(4).onTrue();

    // m_secondaryController.button(5).onTrue();
    m_secondaryController.button(6).onTrue(new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(1.5));
    m_secondaryController.button(7).onTrue(m_ClimberSubsystem.ResetEncoders());
    m_secondaryController.button(8).whileTrue(new ForwardClimb(m_ArmSubsystem, m_ClimberSubsystem));

    m_secondaryController.button(9).onTrue(m_ArmSubsystem.SetPIDPosition(2));
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

    LeftTrigger.whileTrue(new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem));

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

  public final double getLeftTrigger() {
    return m_driverController.getLeftTriggerAxis();
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
}


/**
 * How well does coral identify notes? - Picks up farther notes over closer notes
 * Do trajectories work well with timeouts?
 * Triple note autos - Middle + left, Middle + right
 * Maybe auto that picks up mid notes
 * Climb sequences - done
 * Test auto shooting - works
 * 
 * Find a wait command for commands
 */