// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

//import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.NOTNOTNoteSuck;
import frc.robot.commands.NoteOffIntake;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.ZeroHeading;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Trajectories;
import frc.robot.subsystems.Swerve.DriveSubsystem;

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
  private final Trajectories m_trajectories = new Trajectories(m_robotDrive);
  private final SendableChooser<Command> autoChooser;
  private final Field2d field;

      
  Command forwardTrajectoryBlue, forwardTrajectory2Blue, backwardTrajectoryBlue;
  Command ampTrajectoryBlue, ampNoteTrajectoryBlue, noteAmpTrajectoryBlue, ampDriveTrajectoryBlue;
  Command forwardTrajectory3Blue;

  Command forwardTrajectoryRed, forwardTrajectory2Red, backwardTrajectoryRed;
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
     ArrayList<Trajectory> Auto1Blue = m_trajectories.getMiddleDoubleScore();
     ArrayList<Trajectory> Auto2Blue = m_trajectories.getAmpDoubleScore();
     ArrayList<Trajectory> Auto3Blue = m_trajectories.getDriveForward();

     forwardTrajectoryBlue = GenerateTrajectoryCommand(Auto1Blue.get(0));
     backwardTrajectoryBlue = GenerateTrajectoryCommand(Auto1Blue.get(1));
     forwardTrajectory2Blue = GenerateTrajectoryCommand(Auto1Blue.get(2));

     ampTrajectoryBlue = GenerateTrajectoryCommand(Auto2Blue.get(0));
     ampNoteTrajectoryBlue = GenerateTrajectoryCommand(Auto2Blue.get(1));
     noteAmpTrajectoryBlue = GenerateTrajectoryCommand(Auto2Blue.get(2));
     ampDriveTrajectoryBlue = GenerateTrajectoryCommand(Auto2Blue.get(3));

     forwardTrajectory3Blue = GenerateTrajectoryCommand(Auto3Blue.get(0));

     m_trajectories.CreateTrajectories(true);
     ArrayList<Trajectory> Auto1Red = m_trajectories.getMiddleDoubleScore();
     ArrayList<Trajectory> Auto2Red = m_trajectories.getAmpDoubleScore();
     ArrayList<Trajectory> Auto3Red = m_trajectories.getDriveForward();

     forwardTrajectoryRed = GenerateTrajectoryCommand(Auto1Red.get(0));
     backwardTrajectoryRed = GenerateTrajectoryCommand(Auto1Red.get(1));
     forwardTrajectory2Red = GenerateTrajectoryCommand(Auto1Red.get(2));

     ampTrajectoryRed = GenerateTrajectoryCommand(Auto2Red.get(0));
     ampNoteTrajectoryRed = GenerateTrajectoryCommand(Auto2Red.get(1));
     noteAmpTrajectoryRed = GenerateTrajectoryCommand(Auto2Red.get(2));
     ampDriveTrajectoryRed = GenerateTrajectoryCommand(Auto2Red.get(3));

     forwardTrajectory3Red = GenerateTrajectoryCommand(Auto3Red.get(0));
     
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
        Commands.run(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    autoChooser = new SendableChooser<>(); // Default auto will be `Commands.none()`

    // Blue autos
    autoChooser.addOption("Blue - Double speaker score", new SequentialCommandGroup(
      
      m_ArmSubsystem.SetPIDPosition(35),
      m_ShooterSubsystem.shootNote(4800),
      Commands.waitSeconds(0.5),

      m_ArmSubsystem.SetPIDPosition(2.5),
      Commands.waitSeconds(2),


      m_IntakeSubsystem.RunIntakeCommand(0.6),
      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(0),
      forwardTrajectoryBlue,
      m_IntakeSubsystem.RunIntakeCommand(0),
      m_ArmSubsystem.SetPIDPosition(10),
      new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(0.5),

      m_ArmSubsystem.SetPIDPosition(2.5),
      backwardTrajectoryBlue,

      Commands.waitSeconds(0.5),

      m_ShooterSubsystem.shootNote(4800),
      Commands.waitSeconds(2),
      m_IntakeSubsystem.RunIntakeCommand(0.6),

      Commands.waitSeconds(0.5),
      m_ShooterSubsystem.shootNote(0),
      m_IntakeSubsystem.RunIntakeCommand(0),
      forwardTrajectory2Blue

    ));
    autoChooser.addOption("Blue - Double amp score", new SequentialCommandGroup(
      m_ArmSubsystem.SetPIDPosition(95),  
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
      m_ArmSubsystem.SetPIDPosition(40),
      
      noteAmpTrajectoryBlue,
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
      m_ArmSubsystem.SetPIDPosition(95),  
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
      m_ArmSubsystem.SetPIDPosition(40),
      
      noteAmpTrajectoryRed,
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
    new JoystickButton(m_driverController, 1).whileTrue(RunIntakeCommand(1));
    new JoystickButton(m_driverController, 2).whileTrue(RunIntakeCommand(-0.3));
    new JoystickButton(m_driverController, 3).onTrue(m_ArmSubsystem.RunArm(0.5)).onFalse(m_ArmSubsystem.RunArm(0));
    new JoystickButton(m_driverController, 4).onTrue(m_ArmSubsystem.RunArm(-0.3)).onFalse(m_ArmSubsystem.RunArm(0));
    new JoystickButton(m_driverController, 5).onTrue(m_ShooterSubsystem.shootNote(Constants.ShooterConstants.reverse)).onFalse(m_ShooterSubsystem.shootNote(0));
    new JoystickButton(m_driverController, 6).onTrue(m_ShooterSubsystem.shootNote(5500)).onFalse(m_ShooterSubsystem.shootNote(0));
    new JoystickButton(m_driverController, 7).onTrue(m_ClimberSubsystem.RunClimber(1)).onFalse(m_ClimberSubsystem.RunClimber(0));
    new JoystickButton(m_driverController, 8).onTrue(m_ClimberSubsystem.RunClimber(-1)).onFalse(m_ClimberSubsystem.RunClimber(0));
    new JoystickButton(m_driverController, 10).whileTrue(ZeroHeading());

    m_secondaryController.button(1).onTrue(m_robotDrive.speakerCentering(m_driverController, m_SensorSubsystem)).onFalse(m_robotDrive.regularDrive(m_driverController));
    //m_secondaryController.button(2).onTrue();
    m_secondaryController.button(3).onTrue(new NoteOffIntake(m_ShooterSubsystem, m_IntakeSubsystem, m_SensorSubsystem).withTimeout(1.5));
    //m_secondaryController.button(4).onTrue(new ShootSpeaker(m_ShooterSubsystem, m_IntakeSubsystem, 4000).withTimeout(3.5));

    m_secondaryController.button(5).onTrue(m_ArmSubsystem.SetPIDNOTNOTSensor(m_SensorSubsystem));
    // m_secondaryController.button(6).onTrue(Commands.waitSeconds(0.5).andThen(RunIntakeCommand(0.3)));
    m_secondaryController.button(7).onTrue(m_ClimberSubsystem.ResetEncoders());
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

    BooleanSupplier RightTriggerSupplier = new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        if (m_driverController.getRightTriggerAxis() > 0.5) return true;
        else return false;
      }
    };
    Trigger RightTrigger = new Trigger(RightTriggerSupplier);

    RightTrigger.onTrue(m_ShooterSubsystem.shootNote(5500)).onFalse(m_ShooterSubsystem.shootNote(0));
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