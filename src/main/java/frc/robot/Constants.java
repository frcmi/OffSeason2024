// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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
}
