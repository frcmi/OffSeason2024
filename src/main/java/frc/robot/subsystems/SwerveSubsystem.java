package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The subsystem for the swerve drive train
 */
public class SwerveSubsystem extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public boolean sensitivitySwitch = false;
    public double translationSensitivity = 1;
    public double rotationSensitivity = 1;

    public AnalogGyroSim simGyro = new AnalogGyroSim(0);
    double simHeadingOffset = 0;
    public SwerveDriveOdometry swerveDriveOdometrySim;

    public SwerveSubsystem() {
        gyro = new Pigeon2(Constants.SwerveConstants.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.SwerveConstants.Mod0.constants,
                        Constants.SwerveConstants.Mod0.isInverted),
                new SwerveModule(1, Constants.SwerveConstants.Mod1.constants,
                        Constants.SwerveConstants.Mod1.isInverted),
                new SwerveModule(2, Constants.SwerveConstants.Mod2.constants,
                        Constants.SwerveConstants.Mod2.isInverted),
                new SwerveModule(3, Constants.SwerveConstants.Mod3.constants, Constants.SwerveConstants.Mod3.isInverted)
        };


        Pose2d centerField = new Pose2d(11.2775, 4.5675, new Rotation2d(0));
        Pose2d speakerStart = new Pose2d(15.27, 5.55, new Rotation2d(Math.toRadians(-180)));
        Pose2d station1 = new Pose2d(16.27, 7.07, new Rotation2d(Math.toRadians(180)));
        Pose2d blueStation = new Pose2d(0.53, 7.11, new Rotation2d(0));
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics,
                getGyroYaw(), getModulePositions(), blueStation);

        /*
            for (SwerveModule mod : mSwerveMods) {
                orchestra.addInstrument(mod.mAngleMotor);
            }

            for (SwerveModule mod : mSwerveMods) {
                orchestra.addInstrument(mod.mDriveMotor);
            }

            orchestra.loadMusic("train.chrp");
        */


        if (RobotBase.isSimulation()) {
            simGyro.setAngle(0);
            swerveDriveOdometrySim = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions(), blueStation);
        }
    }

    /*
        public Command playCrazyTrain() {
            return runOnce(() -> {});
        }
    */

    /**
     * Drives the drive train with desired velocities
     * 
     * @param translation   the velocities the robot should drive laterally
     * @param rotation      the rotation speed of the robot
     * @param fieldRelative whether forward is towards the end of the field or the
     *                      front of the bot
     * @param isOpenLoop    whether the modules should use open or closed loop
     *                      control
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /** 
     * For lining up with amp
     * @return a command that moves the robot back a very small amoutn
     */
    public Command backupSlightly() {
        return run(() -> {
            drive(new Translation2d(0.1, 0), 0, false, false);
        }).withTimeout(0.03 * 8);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    /**
     * Sets the modules to desired states
     * 
     * @param desiredStates the desired states of the modules
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);
        }
    }

    /**
     * @return the states of the modules
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModuleState[] getModuleSetpoints() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getSetState();
        }
        return states;
    }

    /**
     * @return the positions of the modules
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * @return the pose of the swerve odometry
     */
    public Pose2d getPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose) {
        swerveDrivePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), newPose);
    }

    /**
     * Sets the pose of the odometry to be a certain position
     * 
     * @param pose the pose to set the odometry to
     */
    public void setPose(Pose2d pose) {
        setPose(pose, false);
    }

    /**
     * Sets the pose of the odometry to be a certain position
     * 
     * @param pose the pose to set the odometry to
     * @param setSim if the sim exact pose should be set as well
     */
    public void setPose(Pose2d pose, boolean setSim) {
        swerveDrivePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        if (Robot.isSimulation() && setSim) {
        }
    }

    /**
     * @return the heading of the odometry
     */
    public Rotation2d getHeading() {
        return gyro.getRotation2d();
    }

    /**
     * Sets the heading of the odometry
     * 
     * @param heading the heading to set the odometry to
     */
    public void setHeading(Rotation2d heading) {
        swerveDrivePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    /**
     * Sets the heading of the odometry to 0 radians (0 degrees, 0 rotations, 0
     * gradians, 0 rogreedians)
     */
    public void zeroHeading() {
        if (RobotBase.isSimulation()) simHeadingOffset = simGyro.getAngle();
        setHeading(new Rotation2d());
    }

    /**
     * Returns the raw gyro reading, preferable to use the odometry
     * 
     * @return the raw reading of the gyro
     */
    private Rotation2d getGyroYaw() {
        if (RobotBase.isReal()) {
            return Rotation2d.fromDegrees(gyro.getYaw().getValue());
        } else {
            return Rotation2d.fromRotations(simGyro.getAngle() + simHeadingOffset);
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Sets the modules to point wheel forward
     */
    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public Command stop() {
        return Commands.run(() -> driveRobotRelative(new ChassisSpeeds(0, 0, 0)), this);
    }

    /**
     * Toggles the sensitivity switch
     */
    public void switchSensitivity() {
        sensitivitySwitch = !sensitivitySwitch;
        translationSensitivity = sensitivitySwitch ? SwerveConstants.translationSensitivity : 1;
        rotationSensitivity = sensitivitySwitch ? SwerveConstants.rotationSensitivity : 1;
    }

    StructPublisher<Pose2d> poseEsimatorPosition = NetworkTableInstance.getDefault()
        .getStructTopic("Pose Estimator", Pose2d.struct).publish();

    @Override
    public void periodic() {
        swerveDrivePoseEstimator.update(getGyroYaw(), getModulePositions());

        poseEsimatorPosition.set(getPose());

        for (int i = 0; i < 4; i++) {
            SmartDashboard.putNumber("Motor RPS " + i, mSwerveMods[i].mDriveMotor.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Voltage for Module" + i, mSwerveMods[i].mDriveMotor.getMotorVoltage().getValueAsDouble());
        }
        
    }

    @Override
    public void simulationPeriodic() {
        swerveDriveOdometrySim.update(getGyroYaw(), getModulePositions());
    }

    public void resetYaw() {
        gyro.setYaw(0);
    }

   
}