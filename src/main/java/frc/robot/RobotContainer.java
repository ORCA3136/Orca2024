// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunShooterCommand;
import frc.robot.commands.RunArmCommand;
import frc.robot.commands.SetSwerveXCommand;
import frc.robot.commands.FollowPathCommand;
import frc.robot.commands.PathfindThenFollowPathCommand;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.ArmPID;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PlaceholderSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

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
  private final PlaceholderSubsystem m_PlaceholderSubsystem = new PlaceholderSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

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

    m_robotDrive.configureAutoBuilder();

    // Starts recording to data log
    //DataLogManager.start();

    // Record both DS control and joystick data
    //DriverStation.startDataLog(DataLogManager.getLog());
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
    new JoystickButton(m_driverController, 4).whileTrue(RunArmCommand(-0.25));
    //new JoystickButton(m_driverController, 5).whileTrue(RunShooterCommand(-0.5));
    //new JoystickButton(m_driverController, 6).whileTrue(RunShooterCommand(1));
    new JoystickButton(m_driverController, 6).whileTrue(RunShooterCommand(0.2));
    new JoystickButton(m_driverController, 8).whileTrue(ZeroHeading());
    
    //PID buttons
    //new JoystickButton(m_driverController, 5).onTrue(new ArmPID(100 * 0.04, m_ArmSubsystem));
    //new JoystickButton(m_driverController, 6).onTrue(new ArmPID(100 * 0.50, m_ArmSubsystem));
    new JoystickButton(m_driverController, 7).onTrue(new ArmPID(m_ArmSubsystem));
    //new JoystickButton(m_driverController, 8).onTrue(new ArmPID(100 * 0.30, m_ArmSubsystem));

    //0.52 Amp --- Intake level 0.3 --- Intake low 0.15 --- 0.01 Floor

    //Pathfinding buttons
    //new JoystickButton(m_driverController, 6).whileTrue(FollowPathCommand("Drive"));
    //new JoystickButton(m_driverController, ).whileTrue(PathfindThenFollowPathCommand("ApproachAmp"));
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }


  //public methods
  public final int getPOV() {
    return m_driverController.getPOV();
  }

  public final void StartPID() {
    
  }


  //private commands
  private final RunIntakeCommand RunIntakeCommand(double speed) {

    return new RunIntakeCommand(speed, m_IntakeSubsystem);

  }

  private final RunShooterCommand RunShooterCommand(double speed) {

    return new RunShooterCommand(-speed, m_ShooterSubsystem);

  }

  private final RunArmCommand RunArmCommand(double speed) {

    m_ArmSubsystem.setInUse();
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
}
