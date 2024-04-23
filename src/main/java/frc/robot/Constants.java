// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.vision.Camera;
import frc.robot.vision.CameraDescription;
import frc.robot.vision.CameraDescription.CameraType;

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
  
  public static class ClimberConstants {
    public static final int kRightClimberId = 50;
    public static final int kLeftClimberId = 51;

    public static final double kClimberUp = 0.3;
    public static final double kClimberDown = -0.7;
  }
  
  public static class AutoConstants {
    public static final double maxSpeed = 2; //mps
    public static final double maxAcceleration = 1; //mpsps
    public static final double maxRotationalSpeed = 1; //rps
    public static final double maxRotationalAcceleration = 0.5; //rpsps
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(1, 0, 0), //translation pid
      new PIDConstants(1, 0, 0), //rotation pid
      maxSpeed, //max speed of module
      0, //TODO: change to drive base radius
      new ReplanningConfig()
    );
  }

  public static class VisionConstants {
    public static final CameraDescription[] kCameras = new CameraDescription[] {
      new CameraDescription("Arducam_OV9281_USB_Camera", CameraType.PHOTONVISION)
    };

    public static final Camera.Specification[] kCameraSpecs = new Camera.Specification[] {
      new Camera.Specification(1280, 800, Rotation2d.fromDegrees(90), 0.44, 0.05, 30, 10, 30)
    };

    public static final Transform3d[] kCameraOffsets = new Transform3d[] {
      new Transform3d(Units.inchesToMeters(5), Units.inchesToMeters(9), Units.inchesToMeters(21.85), new Rotation3d(0, -10.5 * Math.PI / 180, 0))
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
