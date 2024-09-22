// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.SwerveSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...

  public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public static final TalonFX Yippie = new TalonFX(4);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  public static final double kMaxVelocity = SwerveConstants.maxSpeed; // kSpeedAt12VoltsMps desired top speed
  public static final double kMaxAngularVelocity = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(kMaxVelocity * 0.1).withRotationalDeadband(kMaxAngularVelocity * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {    // Configure the trigger bindings
    configureBindings();
    autoChooser = new SendableChooser<>();
    autoChooser.addOption("No Auto", Commands.print("no auto"));
    autoChooser.addOption("Drive Forwards", new TeleopSwerve(
            swerveSubsystem, 
            () -> -1, 
            () -> 0, 
            () -> 0, 
            () -> false,
            () -> false
        )
      .withTimeout(2)
    );
    SmartDashboard.putData("Auto", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // -y on the joystick is up
    // and then we want +x (towards the other side of the field) to be up

    // similarly, -x on the joystick is left
    // and we want +y (the left side of the field) to be to the left

    // positive angular velocity is counterclockwise looking down on the field
    // and we want the robot to rotate counterclockwise when we flick the right
    // joystick left

    // swerveSubsystem.setDefaultCommand(
    //     swerveSubsystem.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * kMaxVelocity)
    //         .withVelocityY(-driverController.getLeftX() * kMaxVelocity)
    //         .withRotationalRate(-driverController.getRightX() * kMaxAngularVelocity)));

    // swerveSubsystem.setDefaultCommand(
    //     new TeleopSwerve(
    //         swerveSubsystem, 
    //         () -> driverController.getLeftY() * swerveSubsystem.translationSensitivity, 
    //         () -> driverController.getLeftX() * swerveSubsystem.translationSensitivity, 
    //         () -> driverController.getRightX() * swerveSubsystem.rotationSensitivity, 
    //         () -> false, //robotCentric.getAsBoolean()
    //         () -> false
    //     )
    // );
    swerveSubsystem.setDefaultCommand(
        new TeleopSwerve(
            swerveSubsystem, 
            () -> driverController.getLeftY() * swerveSubsystem.translationSensitivity, 
            () -> driverController.getLeftX() * swerveSubsystem.translationSensitivity, 
            () -> driverController.getRightX() * swerveSubsystem.rotationSensitivity, 
            () -> false,
            () -> false
        )
    );
    driverController.y().onTrue(Commands.run(() -> swerveSubsystem.resetYaw()));

    driverController.povRight().onTrue(Commands.run(() -> Yippie.setVoltage(14)));
    driverController.povDown().onTrue(Commands.run(() -> Yippie.setVoltage(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

}