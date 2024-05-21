package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

public class AutoAlignCommand extends Command {
    public AutoAlignCommand(ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem) {
        shooter = shooterSubsystem;
        swerve = swerveSubsystem;

        var pidConstants = AutoConstants.pathFollowerConfig.rotationConstants;
        pidController = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);

        swerveRequest = new SwerveRequest.FieldCentric()
                .withRotationalDeadband(RobotContainer.kMaxAngularVelocity * 0.1)
                .withDriveRequestType(DriveRequestType.Velocity);

        addRequirements(swerveSubsystem);
    }

    private ShooterSubsystem shooter;
    private SwerveSubsystem swerve;
    private PIDController pidController;
    private SwerveRequest.FieldCentric swerveRequest;

    public static class AutoAlignResults {
        public Rotation2d shooterAngle;
        public double noteAirTime;
        public Translation3d initialVelocity, finalVelocity;
    }

    /**
     * Computes the angle to shoot a ring at, not accounting for air resistance, but
     * accounting for gravity
     * 
     * @param shooterToTarget The vector from the shooter to the target in METERS
     *                        (e.g. targetPose.minus(shooterPose))
     * @param initialVelocity The velocity that the shooter would shoot at in METERS
     *                        PER SECOND
     * @return The angle to shoot at, if possible - along with other contextual
     *         values
     */
    public static Optional<AutoAlignResults> computeShooterAngle(Translation3d shooterToTarget,
            double initialVelocity) {
        // -delta_y + v_not * t * sin(theta) - 1/2 * g * t^2 = -delta_x + v_not * t *
        // cos(theta)

        // delta_x - delta_y + v_not * t * (sin(theta) - cos(theta)) - 1/2 * g * t^2 = 0

        // t = (v_not * (sin(theta) - cos(theta)) +/- sqrt(v_not^2 * (sin(theta) -
        // cos(theta))^2 - 2 * g * (delta_y - delta_x)))/g

        // => v_not^2 * (sin(theta) - cos(theta))^2 - 2 * g * (delta_y - delta_x) >= 0
        // v_not^2 * (sin(theta) - cos(theta))^2 >= 2 * g * (delta_y - delta_x)
        // v_not * (sin(theta) - cos(theta)) >= sqrt(2 * g * (delta_y - delta_x))
        // sin(theta) - cos(theta) >= 1/v_not * sqrt(2 * g * (delta_y - delta_x))

        // sin(theta) - cos(theta) = x
        // sin(a + b) = sin(a) * cos(b) + cos(a) + sin(b)
        // a = theta
        // b = -pi/4
        // sin(b) = -sqrt(2)/2
        // cos(b) = sqrt(2)/2

        // multiply both sides by sqrt(2)/2
        // sqrt(2)/2 * x = sqrt(2)/2 * sin(theta) - sqrt(2)/2 * cos(theta)
        // sqrt(2)/2 * x = sin(theta - pi/4)
        // asin(sqrt(2)/2 * x) = theta - pi/4
        // theta = asin(sqrt(2)/2 * x) + pi/4
        // THEREFORE theta >= asin(sqrt(2)/2 * (1/v_not * sqrt(2 * g * (delta_y -
        // delta_x)))) + pi/4
        // theta >= asin(1/v_not * sqrt(g * (delta_y - delta_x))) + pi/4

        double deltaX = shooterToTarget.toTranslation2d().getNorm();
        double deltaY = shooterToTarget.getZ();
        double g = 9.81;

        double pitch = Math.asin(Math.sqrt(g * (deltaY - deltaX)) / initialVelocity) + Math.PI / 4;
        double cosPitch = Math.cos(pitch);
        double sinPitch = Math.sin(pitch);

        // because we know the sqrt term is 0, we can just ignore it. its faster :)
        double t = (initialVelocity * (Math.sin(pitch) - Math.cos(pitch))) / g;
        
        double cosYaw = shooterToTarget.getX() / deltaX;
        double sinYaw = shooterToTarget.getY() / deltaX;

        // i wish i could use energy conservation but that only deals with magnitudes
        // velocityDirection will always have a length of 1
        var velocityDirection = new Translation3d(cosYaw * cosPitch, sinYaw * cosPitch, sinPitch);
        var initialVelocityVector = velocityDirection.times(initialVelocity);
        var gravityVector = new Translation3d(0, 0, -g);
        var finalVelocityVector = initialVelocityVector.plus(gravityVector.times(t));

        double velocityX = finalVelocityVector.toTranslation2d().getNorm();
        double velocityY = finalVelocityVector.getZ();
        double finalVelocityAngle = Math.atan2(velocityY, velocityX);

        if (finalVelocityAngle < ShooterConstants.kMinVelocityAngle) {
            return Optional.empty();
        }

        var results = new AutoAlignResults();
        results.shooterAngle = new Rotation2d(pitch);
        results.noteAirTime = t;
        results.initialVelocity = initialVelocityVector;
        results.finalVelocity = finalVelocityVector;

        return Optional.of(results);
    }

    @Override
    public void execute() {
        var pose = swerve.getState().Pose;
        var shooterPose = new Pose3d(pose).plus(ShooterConstants.kRobotToPivot);

        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        var speakerPose = alliance == DriverStation.Alliance.Blue ? ShooterConstants.kBlueSpeaker
                : ShooterConstants.kRedSpeaker;

        var shooterToSpeaker = speakerPose.minus(shooterPose).getTranslation();
        var targetRotation = new Rotation2d(shooterToSpeaker.getX(), shooterToSpeaker.getY());
        var currentRotation = shooterPose.getRotation().toRotation2d();

        double pidOutput = pidController.calculate(currentRotation.getRadians(), targetRotation.getRadians());
        double angularVelocity = MathUtil.clamp(pidOutput, -1, 1) * AutoConstants.kMaxAngularVelocity;

        swerve.applyRequest(() -> {
            if (DriverStation.isTeleopEnabled()) {
                // see RobotContainer
                var controller = RobotContainer.driverController;
                swerveRequest.VelocityX = -controller.getLeftY() * RobotContainer.kMaxVelocity;
                swerveRequest.VelocityY = -controller.getLeftX() * RobotContainer.kMaxVelocity;
            }

            swerveRequest.RotationalRate = angularVelocity;
            return swerveRequest;
        });

        var result = computeShooterAngle(shooterToSpeaker, ShooterConstants.kInitialNoteVelocity);
        if (result.isPresent()) {
            double shooterAngle = result.get().shooterAngle.getRadians();
            shooterAngle = MathUtil.clamp(shooterAngle, ShooterConstants.kMinPosition, ShooterConstants.kMaxPosition);
            
            shooter.setGoal(shooterAngle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.doStop();
    }
}
