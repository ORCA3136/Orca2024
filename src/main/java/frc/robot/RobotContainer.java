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

      
  Command forwardTrajectoryBlue1, farForwardTrajectoryBlue, backwardTrajectoryBlue1;
  Command forwardTrajectoryBlue2, backwardTrajectoryBlue2, speakAmpNoteTrajectoryBlue, ampNoteSpeakTrajectoryBlue, speakSourceNoteTrajectoryBlue, sourceNoteSpeakTrajectoryBlue;
  Command ampTrajectoryBlue, ampNoteTrajectoryBlue, noteAmpTrajectoryBlue, ampDriveTrajectoryBlue;
  Command forwardTrajectory3Blue;

  Command forwardTrajectoryRed, forwardTrajectory2Red, backwardTrajectoryRed;
  Command speakAmpNoteTrajectoryRed, ampNoteSpeakTrajectoryRed, speakSourceNoteTrajectoryRed, sourceNoteSpeakTrajectoryRed;
  Command ampTrajectoryRed, ampNoteTrajectoryRed, noteAmpTrajectoryRed, ampDriveTrajectoryRed;
  Command forwardTrajectory3Red;

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

     m_trajectories.CreateTrajectories(false);
     ArrayList<Trajectory> BlueMidSpeak = m_trajectories.getMiddleDoubleScore();
     ArrayList<Trajectory> BlueDubAmp = m_trajectories.getAmpDoubleScore();
     ArrayList<Trajectory> BlueDriveOut = m_trajectories.getDriveForward();
     ArrayList<Trajectory> BlueTripSpeak = m_trajectories.getTripleSpeakerScore();

     forwardTrajectoryBlue1 = GenerateTrajectoryCommand(BlueMidSpeak.get(0));
     backwardTrajectoryBlue1 = GenerateTrajectoryCommand(BlueMidSpeak.get(1));
     farForwardTrajectoryBlue = GenerateTrajectoryCommand(BlueMidSpeak.get(2));

     forwardTrajectoryBlue2 = GenerateTrajectoryCommand(BlueMidSpeak.get(0));
     backwardTrajectoryBlue2 = GenerateTrajectoryCommand(BlueMidSpeak.get(1));
     speakAmpNoteTrajectoryBlue = GenerateTrajectoryCommand(BlueTripSpeak.get(0));
     ampNoteSpeakTrajectoryBlue = GenerateTrajectoryCommand(BlueTripSpeak.get(1));
     speakSourceNoteTrajectoryBlue = GenerateTrajectoryCommand(BlueTripSpeak.get(2));
     sourceNoteSpeakTrajectoryBlue = GenerateTrajectoryCommand(BlueTripSpeak.get(3));

     ampTrajectoryBlue = GenerateTrajectoryCommand(BlueDubAmp.get(0));
     ampNoteTrajectoryBlue = GenerateTrajectoryCommand(BlueDubAmp.get(1));
     noteAmpTrajectoryBlue = GenerateTrajectoryCommand(BlueDubAmp.get(2));
     ampDriveTrajectoryBlue = GenerateTrajectoryCommand(BlueDubAmp.get(3));

     forwardTrajectory3Blue = GenerateTrajectoryCommand(BlueDriveOut.get(0));

     m_trajectories.CreateTrajectories(true);
     ArrayList<Trajectory> RedMidSpeak = m_trajectories.getMiddleDoubleScore();
     ArrayList<Trajectory> RedDubAmp = m_trajectories.getAmpDoubleScore();
     ArrayList<Trajectory> RedDriveOut = m_trajectories.getDriveForward();
     ArrayList<Trajectory> RedTripSpeak = m_trajectories.getTripleSpeakerScore();

     forwardTrajectoryRed = GenerateTrajectoryCommand(RedMidSpeak.get(0));
     backwardTrajectoryRed = GenerateTrajectoryCommand(RedMidSpeak.get(1));
     forwardTrajectory2Red = GenerateTrajectoryCommand(RedMidSpeak.get(2));

     speakAmpNoteTrajectoryRed = GenerateTrajectoryCommand(RedTripSpeak.get(0));
     ampNoteSpeakTrajectoryRed = GenerateTrajectoryCommand(RedTripSpeak.get(1));
     speakSourceNoteTrajectoryRed = GenerateTrajectoryCommand(RedTripSpeak.get(2));
     sourceNoteSpeakTrajectoryRed = GenerateTrajectoryCommand(RedTripSpeak.get(3));

     ampTrajectoryRed = GenerateTrajectoryCommand(RedDubAmp.get(0));
     ampNoteTrajectoryRed = GenerateTrajectoryCommand(RedDubAmp.get(1));
     noteAmpTrajectoryRed = GenerateTrajectoryCommand(RedDubAmp.get(2));
     ampDriveTrajectoryRed = GenerateTrajectoryCommand(RedDubAmp.get(3));

     forwardTrajectory3Red = GenerateTrajectoryCommand(RedDriveOut.get(0));
     
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
      
      m_ShooterSubsystem.shootNote(1000),
      Commands.waitSeconds(0.5),

      m_ArmSubsystem.SetPIDPosition(35),
      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(0.5),

      m_ArmSubsystem.SetPIDPosition(1.5),
      Commands.waitSeconds(1.5),


      m_IntakeSubsystem.RunIntakeCommand(0.6),
      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(0),
      forwardTrajectoryBlue2.withTimeout(1.5),
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(10),
      new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),

      m_ArmSubsystem.SetPIDPosition(2),
      backwardTrajectoryBlue2,

      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(2),
      m_IntakeSubsystem.RunIntakeCommand(0.6),

      Commands.waitSeconds(0.5),
      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0.3),
      speakAmpNoteTrajectoryBlue,

      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(10),
      new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),

      m_ArmSubsystem.SetPIDPosition(2),
      ampNoteSpeakTrajectoryBlue,

      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(2),
      m_IntakeSubsystem.RunIntakeCommand(0.6),

      Commands.waitSeconds(0.5),
      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0)

    ));
    autoChooser.addOption("Blue - Double speaker score", new SequentialCommandGroup(
      
      m_ShooterSubsystem.shootNote(1000),
      Commands.waitSeconds(0.5),

      m_ArmSubsystem.SetPIDPosition(35),
      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(0.5),

      m_ArmSubsystem.SetPIDPosition(1.5),
      Commands.waitSeconds(1.5),


      m_IntakeSubsystem.RunIntakeCommand(0.6),
      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(0),
      forwardTrajectoryBlue1.withTimeout(1.5),
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(10),
      new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),

      m_ArmSubsystem.SetPIDPosition(2),
      backwardTrajectoryBlue1,

      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(5500),
      Commands.waitSeconds(2),
      m_IntakeSubsystem.RunIntakeCommand(0.6),

      Commands.waitSeconds(0.5),
      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0),
      farForwardTrajectoryBlue

    ));
    autoChooser.addOption("Blue - Double amp score", new SequentialCommandGroup(
      m_ArmSubsystem.SetPIDPosition(90),  
      ampTrajectoryBlue,
      
      m_ShooterSubsystem.shootNote(700),

      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(0),
      m_ArmSubsystem.SetPIDPosition(45),

      Commands.waitSeconds(0.5),
      m_ArmSubsystem.SetPIDPosition(3),
      Commands.waitSeconds(1),
      m_IntakeSubsystem.RunIntakeCommand(0.3),

      ampNoteTrajectoryBlue,
      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem),
      m_ArmSubsystem.SetPIDPosition(40),
      
      noteAmpTrajectoryBlue,
      m_IntakeSubsystem.RunIntakeCommand(0.3),
      m_ArmSubsystem.SetPIDPosition(90),
      Commands.waitSeconds(1),
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ShooterSubsystem.shootNote(700),
      Commands.waitSeconds(0.5),
      
      m_ShooterSubsystem.shootNote(0),
      m_ArmSubsystem.SetPIDPosition(60),
      m_IntakeSubsystem.RunIntakeCommand(0),
      Commands.waitSeconds(0.25),
      m_ArmSubsystem.SetPIDPosition(25),
      Commands.waitSeconds(0.25),

      ampDriveTrajectoryBlue
    )); 
    autoChooser.addOption("Blue - Drive forward", new SequentialCommandGroup(
      m_ArmSubsystem.SetPIDPosition(35),
      m_ShooterSubsystem.shootNote(4800),
      Commands.waitSeconds(0.5),

      m_ArmSubsystem.SetPIDPosition(2.5),
      Commands.waitSeconds(2),


      m_IntakeSubsystem.RunIntakeCommand(0.6),
      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(25),
    
      forwardTrajectory3Blue
    ));

    // Red autos
    autoChooser.addOption("Red - Double speaker score", new SequentialCommandGroup(
      
      m_ArmSubsystem.SetPIDPosition(35),
      m_ShooterSubsystem.shootNote(5200),
      Commands.waitSeconds(0.5),

      m_ArmSubsystem.SetPIDPosition(2.5),
      Commands.waitSeconds(2),


      m_IntakeSubsystem.RunIntakeCommand(0.6),
      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(0),
      forwardTrajectoryRed,
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(10),
      new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),

      m_ArmSubsystem.SetPIDPosition(2.5),
      backwardTrajectoryRed,

      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(4800),
      Commands.waitSeconds(2),
      m_IntakeSubsystem.RunIntakeCommand(0.6),

      Commands.waitSeconds(0.5),
      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0),
      forwardTrajectory2Red

    ));
    autoChooser.addOption("Red - Double amp score", new SequentialCommandGroup(
      m_ArmSubsystem.SetPIDPosition(90),  
      ampTrajectoryRed,
      
      m_ShooterSubsystem.shootNote(700),

      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(0),
      m_ArmSubsystem.SetPIDPosition(45),

      Commands.waitSeconds(0.5),
      m_ArmSubsystem.SetPIDPosition(3),
      Commands.waitSeconds(1),
      m_IntakeSubsystem.RunIntakeCommand(0.3),

      ampNoteTrajectoryRed,
      new NOTNOTNoteSuck(m_robotDrive, m_IntakeSubsystem, m_SensorSubsystem, m_ShooterSubsystem).withTimeout(0.5),
      m_ArmSubsystem.SetPIDPosition(40),
      
      noteAmpTrajectoryRed,
      m_ArmSubsystem.SetPIDPosition(90),
      Commands.waitSeconds(1),
      m_ShooterSubsystem.shootNote(700),
      Commands.waitSeconds(0.5),
      
      m_ShooterSubsystem.shootNote(0),
      m_ArmSubsystem.SetPIDPosition(60),
      m_IntakeSubsystem.RunIntakeCommand(0),
      Commands.waitSeconds(0.5),
      m_ArmSubsystem.SetPIDPosition(25),
      Commands.waitSeconds(1),

      ampDriveTrajectoryRed
    )); 
    autoChooser.addOption("Red - Drive forward", new SequentialCommandGroup(
      m_ArmSubsystem.SetPIDPosition(35),
      m_ShooterSubsystem.shootNote(5200),
      Commands.waitSeconds(0.5),

      m_ArmSubsystem.SetPIDPosition(2.5),
      Commands.waitSeconds(2),


      m_IntakeSubsystem.RunIntakeCommand(0.6),
      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(25),
    
      forwardTrajectory3Red
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

    // Maybe auto sensing note for off fly subroutine
    new JoystickButton(m_driverController, 1).whileTrue(new RunIntakeCommand(1, m_IntakeSubsystem)).onFalse(new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(1.5));
    // Add in slow flywheel speed
    new JoystickButton(m_driverController, 2).whileTrue(new ParallelCommandGroup(new RunIntakeCommand(-0.3, m_IntakeSubsystem), m_ShooterSubsystem.shootNote(Constants.ShooterConstants.reverse))).onFalse(m_ShooterSubsystem.shootNote(0));

    // Unused - besides climb
    new JoystickButton(m_driverController, 3).onTrue(m_ArmSubsystem.RunArm(0.5)).onFalse(m_ArmSubsystem.RunArm(0));
    new JoystickButton(m_driverController, 4).onTrue(m_ArmSubsystem.RunArm(-0.3)).onFalse(m_ArmSubsystem.RunArm(0));

    // Go to 2
    // new JoystickButton(m_driverController, 5).onTrue(m_ShooterSubsystem.shootNote(Constants.ShooterConstants.reverse)).onFalse(m_ShooterSubsystem.shootNote(0));
    // Manual shoot
    new JoystickButton(m_driverController, 6).onTrue(m_ShooterSubsystem.shootNote(5500)).onFalse(m_ShooterSubsystem.shootNote(0));

    // Maybe front climb routine
    new JoystickButton(m_driverController, 7).onTrue(m_ClimberSubsystem.RunClimber(1)).onFalse(m_ClimberSubsystem.RunClimber(0));
    // Maybe back/harmony climb routine
    new JoystickButton(m_driverController, 8).onTrue(m_ClimberSubsystem.RunClimber(-1)).onFalse(m_ClimberSubsystem.RunClimber(0));
    
    new JoystickButton(m_driverController, 10).whileTrue(new ZeroHeading(m_robotDrive));

    //---------------------------------------------------------------------------------------------------------------------------------

    // m_secondaryController.button(1).onTrue(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem)).onFalse(m_robotDrive.regularDrive(m_driverController));
    // m_secondaryController.button(2).onTrue(m_ArmSubsystem.SetPIDSensor(m_SensorSubsystem));
    // m_secondaryController.button(3).onTrue();
    m_secondaryController.button(4).whileTrue(new ForwardClimb(m_ArmSubsystem, m_ClimberSubsystem));

    // m_secondaryController.button(5).onTrue();
    m_secondaryController.button(6).onTrue(new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(1.5));
    m_secondaryController.button(7).onTrue(m_ClimberSubsystem.ResetEncoders());
    // m_secondaryController.button(8).onTrue();

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
 * Climb sequences - start works
 * Test auto shooting - works
 * 
 * Find a wait command for commands
 */