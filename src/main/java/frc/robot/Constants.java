// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public final class variableShooterConstants {
    public static final double kMaxShooterPositionVolts = 0;
    public static final int kVariableShooterMotorId = 0;
    public static final int kVariableShooterEncoderId = 0;
    public static final double kTorqueShooterConstant = 0;

    // PID stuff
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    // feed forward stuff
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // motion stuff
    public static final double kMaxLiftVelocity = 0;
    public static final double kMaxLiftAcceleration = 0;
    public static final double kDefaultSpeed = 0;

    // angle stuff
    public static final double kMaxPosition = 0;
    public static final double kMinPosition = 0;
    public static final double kShootAngle = 0;
    public static final double kGravityLimit = 0;

    public static final double kShooterEncoderOffset = 0;

    // raising and lowering stuff

    public static final double kRaiseShooterVolts = 0;
    public static final double klowerShooterVolts = 0;
    public static final double kShooterCurrentLimit = 0;
  }
}
