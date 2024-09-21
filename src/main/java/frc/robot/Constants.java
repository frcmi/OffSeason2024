// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.robot.vision.Camera;
import frc.robot.vision.CameraDescription;
import frc.robot.vision.CameraDescription.CameraType;

import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double stickDeadband = 0.1;
  }

  public final class ShooterConstants {
    public static final double kMaxShooterPositionVolts = 0;
    public static final int kVariableShooterMotorId = 0;
    public static final int kVariableShooterEncoderId = 0;
    public static final double kTorqueShooterConstant = 0;

    // PID constants
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    // feed forward constants
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // motion constants
    public static final double kMaxLiftVelocity = 0;
    public static final double kMaxLiftAcceleration = 0;
    public static final double kDefaultSpeed = 0;

    // angle constants
    public static final double kMaxPosition = 0;
    public static final double kMinPosition = 0;
    public static final double kMinVelocityAngle = 0;
    public static final double kGravityLimit = 0;

    public static final double kShooterEncoderOffset = 0;
    public static final double kShooterCurrentLimit = 0;

    // auto align constants
    public static final Transform3d kRobotToPivot = new Transform3d();
    public static final Pose3d kRedSpeaker = new Pose3d();
    public static final Pose3d kBlueSpeaker = new Pose3d();

    // IMPORTANT FOR AUTO ALIGN
    public static final double kInitialNoteVelocity = 0;
  }

  public static final class SwerveConstants {
    public static final double kAllowedDistanceToDestination = 0.1;
    public static final double kAllowedRotationDifferenceToDestination = 0.1; // Radians

    public static final int pigeonID = 0;

    public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(28);
    public static final double wheelBase = Units.inchesToMeters(28);
    public static final double wheelCircumference = chosenModule.wheelCircumference;
    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
        //Jett was here

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    public static final float currentLimitModifier = 0.1f;

    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = (int)(25 * currentLimitModifier);
    public static final int angleCurrentThreshold = (int)(40 * currentLimitModifier);
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = (int)(35 * currentLimitModifier);
    public static final int driveCurrentThreshold = (int)(60 * currentLimitModifier);
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.02;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
    public static final double driveKV = 1.51;
    public static final double driveKA = 0.27;

    /* Swerve Profiling Values */
    /** Volts (out of 12) */
    public static final double maxSpeed = 12.9; // TODO: This must be tuned to specific robot
    /** Volts (out of 12) */
    public static final double maxAngularVelocity = 12; // TODO: This must be tuned to specific robot

    /* Sensitivity Values */
    public static final double translationSensitivity = 0.75;
    public static final double rotationSensitivity = 0.5;

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 9; //
      public static final Rotation2d angleOffset = Rotation2d.fromRadians(2.679864436436215);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final boolean isInverted = true;
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromRadians(0.268446637879987);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final boolean isInverted = true;
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromRadians(0.02454369260617);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final boolean isInverted = true;
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 19;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromRadians(0.846);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final boolean isInverted = true;
    }
  }

  public static class ClimberConstants {
    public static final int kRightClimberId = 50;
    public static final int kLeftClimberId = 51;

    public static final double kClimberUp = 0.3;
    public static final double kClimberDown = -0.7;
  }

  public static class AutoConstants {
    public static final double kMaxVelocity = 2; // m/s
    public static final double kMaxAcceleration = 1; // m/s^2
    public static final double kMaxAngularVelocity = 1; // rad/s
    public static final double kMaxAngularAcceleration = 0.5; // rad/s^2
  }

  public static class VisionConstants {
    public static final CameraDescription[] kCameras = new CameraDescription[] {
        new CameraDescription("Arducam_OV9281_USB_Camera", CameraType.PHOTONVISION)
    };

    public static final Camera.Specification[] kCameraSpecs = new Camera.Specification[] {
        new Camera.Specification(1280, 800, Rotation2d.fromDegrees(90), 0.44, 0.05, 30, 10, 30)
    };

    public static final Transform3d[] kCameraOffsets = new Transform3d[] {
        new Transform3d(Units.inchesToMeters(5), Units.inchesToMeters(9), Units.inchesToMeters(21.85),
            new Rotation3d(0, -10.5 * Math.PI / 180, 0))
    };

    public static final PoseStrategy kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    public static final PoseStrategy kBackupStrategy = PoseStrategy.LOWEST_AMBIGUITY;
    public static final double kMaxAmbiguity = 0.7;
    public static final double kMaxDistance = Units.feetToMeters(10);
  }

  public static class TelemetryConstants {
    public static final boolean kLoggingEnabled = true;
    public static final boolean kDisableNetworkLog = false;
    public static final boolean kDisableDataLog = false;
    public static final String kTabPrefix = "Log";
    public static final double kFMSCheckDelayMillis = 1000;
  }
}
