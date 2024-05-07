package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {
    private Notifier simNotifier = null;
    private double lastSimTime;

    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Robot.isSimulation()) {
            startSimThread();
        }
    }
    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Robot.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });

        simNotifier.startPeriodic(SwerveConstants.kSimLoopPeriod);
    }
}
