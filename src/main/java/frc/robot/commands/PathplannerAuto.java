package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.subsystems.SwerveSubsystem;

public class PathplannerAuto {
    public static void InitializePathPlanner(SwerveSubsystem swerveSubsystem) {
        AutoBuilder.configureHolonomic(
            swerveSubsystem::getPose, 
            swerveSubsystem::resetPose, 
            swerveSubsystem::getChassisSpeeds,
            swerveSubsystem::driveRobotRelative, 
            new HolonomicPathFollowerConfig(2, 0.43,
            new ReplanningConfig(true, false)),
            () -> false,
            swerveSubsystem
        );
    }
}
