// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveSubsystem;

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
    }

    public static class SwerveConstants {
        public static final double kSimLoopPeriod = 0.005; // 5 ms

        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with
        // the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        public static final Slot0Configs kSteerGains = new Slot0Configs()
                .withKP(100).withKI(0).withKD(0.2)
                .withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        public static final Slot0Configs kDriveGains = new Slot0Configs()
                .withKP(3).withKI(0).withKD(0)
                .withKS(0).withKV(0).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        public static final double kSlipCurrentA = 300.0;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = 4.73;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        public static final double kCoupleRatio = 3.5714285714285716;

        public static final double kDriveGearRatio = 6.746031746031747;
        public static final double kSteerGearRatio = 21.428571428571427;
        public static final double kWheelRadiusInches = 2;

        public static final boolean kSteerMotorReversed = false;
        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;

        public static final String kCANbusName = "";
        public static final int kPigeonId = 0;

        // These are only used for simulation
        public static final double kSteerInertia = 0.00001;
        public static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        public static final double kSteerFrictionVoltage = 0.25;
        public static final double kDriveFrictionVoltage = 0.25;

        public static final SwerveDrivetrainConstants kDrivetrainConstants = new SwerveDrivetrainConstants()
                .withPigeon2Id(kPigeonId)
                .withCANbusName(kCANbusName);

        public static final SwerveModuleConstantsFactory kConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withWheelRadius(kWheelRadiusInches)
                .withSlipCurrent(kSlipCurrentA)
                .withSteerMotorGains(kSteerGains)
                .withDriveMotorGains(kDriveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage)
                .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                .withCouplingGearRatio(kCoupleRatio)
                .withSteerMotorInverted(kSteerMotorReversed);

        // Front Left
        public static final int kFrontLeftDriveMotorId = 1;
        public static final int kFrontLeftSteerMotorId = 5;
        public static final int kFrontLeftEncoderId = 9;
        public static final double kFrontLeftEncoderOffset = 0.49658203125;

        public static final double kFrontLeftXPosInches = 11.375;
        public static final double kFrontLeftYPosInches = 11.44;

        // Front Right
        public static final int kFrontRightDriveMotorId = 2;
        public static final int kFrontRightSteerMotorId = 6;
        public static final int kFrontRightEncoderId = 10;
        public static final double kFrontRightEncoderOffset = 0.311767578125;

        public static final double kFrontRightXPosInches = 11.375;
        public static final double kFrontRightYPosInches = -11.44;

        // Back Left
        public static final int kBackLeftDriveMotorId = 3;
        public static final int kBackLeftSteerMotorId = 7;
        public static final int kBackLeftEncoderId = 11;
        public static final double kBackLeftEncoderOffset = -0.211669921875;

        public static final double kBackLeftXPosInches = -11.375;
        public static final double kBackLeftYPosInches = 11.44;

        // Back Right
        public static final int kBackRightDriveMotorId = 4;
        public static final int kBackRightSteerMotorId = 8;
        public static final int kBackRightEncoderId = 12;
        public static final double kBackRightEncoderOffset = 0.0732421875;

        public static final double kBackRightXPosInches = -11.375;
        public static final double kBackRightYPosInches = -11.44;

        public static final SwerveModuleConstants kFrontLeft = kConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
                kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches),
                Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
        public static final SwerveModuleConstants kFrontRight = kConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
                kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches),
                Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
        public static final SwerveModuleConstants kBackLeft = kConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId,
                kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches),
                Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
        public static final SwerveModuleConstants kBackRight = kConstantCreator.createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId,
                kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches),
                Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);
    }
}
