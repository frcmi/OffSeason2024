// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command driveToPosition(Pose2d target) {
    return AutoBuilder.pathfindToPose(
        target,
        new PathConstraints(AutoConstants.kMaxVelocity,
            AutoConstants.kMaxAcceleration,
            AutoConstants.kMaxAngularVelocity,
            AutoConstants.kMaxAngularAcceleration));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
