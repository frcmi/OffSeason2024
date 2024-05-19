// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
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
  public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

  public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(SwerveConstants.kDrivetrainConstants,
      SwerveConstants.kFrontLeft, SwerveConstants.kFrontRight,
      SwerveConstants.kBackLeft, SwerveConstants.kBackRight);

  public static final VisionSubsystem visionSubsystem = new VisionSubsystem();

  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  public static final double kMaxVelocity = SwerveConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
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
  public RobotContainer() {
    configureAutoBuilder();
    // Configure the trigger bindings
    configureBindings();
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
    // and we want the robot to rotate counterclockwise when we flick the right joystick left

    swerveSubsystem.setDefaultCommand(
        swerveSubsystem.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * kMaxVelocity)
            .withVelocityY(-driverController.getLeftX() * kMaxVelocity)
            .withRotationalRate(-driverController.getRightX() * kMaxAngularVelocity)
        ));

    driverController.a().whileTrue(swerveSubsystem.applyRequest(() -> brake));
    driverController.b().whileTrue(swerveSubsystem
        .applyRequest(() -> point
            .withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(swerveSubsystem.runOnce(() -> swerveSubsystem.seedFieldRelative()));

    if (Robot.isSimulation()) {
      swerveSubsystem.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());

    driverController.leftTrigger().whileTrue(new AutoAlignCommand(shooterSubsystem, swerveSubsystem));
  }

  private void configureAutoBuilder() {
    // TODO: all this needs to be filled in with methods from DT subsystem...
    AutoBuilder.configureHolonomic(
        () -> swerveSubsystem.getState().Pose,
        null,
        () -> swerveSubsystem.getState().speeds,
        null,
        AutoConstants.pathFollowerConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent())
            return alliance.get() == DriverStation.Alliance.Red;

          System.out.println("Could not obtain alliance from Driver Station!");
          return false;
        },
        null);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    registerCommands();
  }

  private void registerCommands() {
    NamedCommands.registerCommand("auto-align", new AutoAlignCommand(shooterSubsystem, swerveSubsystem));
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