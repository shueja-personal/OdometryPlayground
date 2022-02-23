// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.util.OdometryManager;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /*
  DrivebaseS m_drivebase = new DrivebaseS();
  TurretS m_turret = new TurretS();
  ShooterS m_shooter = new ShooterS();
  */

  @Log
  Field2d m_field = new Field2d();

  /**
   * Change this parameter for the current pose in meters from your odometry.
   * 
   */
  private final OdometryManager m_odometryManager = new OdometryManager(m_field::getRobotPose); //m_drivebaseS::getRobotPose

  @Log
  Command addMeasurementCommand = new InstantCommand(
        ()->{m_odometryManager.addVisionMeasurement(2, Math.PI/4);});

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_field.setRobotPose(new Pose2d(new Translation2d(1, 1), new Rotation2d()));
    // Configure the button bindings
    configureButtonBindings();
  }

  public void robotPeriodic() {
    m_odometryManager.periodic();
    m_field.getObject("target").setPose(
        m_odometryManager.getTargetPose()
      );
    /*
    m_turretS.trackTo(m_odometryManager.getRotationOffset());
    m_shooterS.setSpeed(ShooterS.getRPMForDistance(m_odometryManager.getDistance()));
     */
 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new WaitCommand(0);
  }
}
