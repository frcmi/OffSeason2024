package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
    public final CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.kVariableShooterMotorId,
            MotorType.kBrushless);
    private final DutyCycleEncoder shooterEncoder = new DutyCycleEncoder(ShooterConstants.kVariableShooterEncoderId);
    private final ProfiledPIDController pidController = new ProfiledPIDController(ShooterConstants.kP,
            ShooterConstants.kI, ShooterConstants.kD,
            new TrapezoidProfile.Constraints(ShooterConstants.kMaxLiftVelocity, ShooterConstants.kMaxLiftAcceleration));

    private final ArmFeedforward feedforward = new ArmFeedforward(ShooterConstants.kS, ShooterConstants.kV,
            ShooterConstants.kA);

    public ShooterSubsystem() {
        shooterEncoder.setDistancePerRotation(1);
        shooterEncoder.setPositionOffset(ShooterConstants.kShooterEncoderOffset / 360);
        shooterMotor.setIdleMode(IdleMode.kBrake);

        shooterMotor.setInverted(false);

        setDefaultCommand(stop());
    }

    /**
     * Computes the angle to shoot a ring at, not accounting for air resistance, but accounting for gravity
     * @param shooterToTarget The vector from the shooter to the target in METERS (e.g. targetPose.minus(shooterPose))
     * @param initialVelocity The velocity that the shooter would shoot at in METERS PER SECOND
     * @return The angle to shoot at, if possible
     */
    public static Optional<Rotation2d> computeShooterAngle(Translation3d shooterToTarget, double initialVelocity) {
        // 0 = -delta_y + v_not * t * sin(theta) - 1/2 * g * t^2
        // t = (v_not * t * sin(theta) +/- sqrt(v_not^2 * sin^2(theta) - 2 * g *
        // delta_y)) / g
        // => v_not^2 * sin^2(theta) - 2 * g * delta_y >= 0
        // v_not^2 * sin^2(theta) >= 2 * g * delta_y
        // v_not * sin(theta) >= sqrt(2 * g * delta_y)
        // sin(theta) >= 1/v_not * sqrt(2 * g * delta_y)
        // theta >= asin(1/v_not * sqrt(2 * g * delta_y))

        // -delta_y + v_not * t * sin(theta) - 1/2 * g * t^2 = -delta_x + v_not * t *
        // cos(theta)
        // delta_x - delta_y + v_not * t * (sin(theta) - cos(theta)) - 1/2 * g * t^2 = 0
        // t = (v_not * (sin(theta) - cos(theta)) +/- sqrt(v_not^2 * (sin(theta) -
        // cos(theta))^2 - 2 * g * (delta_y - delta_x)))/g
        // => v_not^2 * (sin(theta) - cos(theta))^2 - 2 * g * (delta_y - delta_x) >= 0
        // v_not^2 * (sin(theta) - cos(theta))^2 >= 2 * g * (delta_y - delta_x)
        // v_not * (sin(theta) - cos(theta)) >= sqrt(2 * g * (delta_y - delta_x))
        // sin(theta) - cos(theta) >= 1/v_not * sqrt(2 * g * (delta_y - delta_x))

        double delta_x = shooterToTarget.toTranslation2d().getNorm();
        double delta_y = shooterToTarget.getZ();
        double g = 9.81;

        double theta = Math.asin(1 / initialVelocity * Math.sqrt(2 * g * delta_y));
        if (Math.sin(theta) - Math.cos(theta) < 1 / initialVelocity * Math.sqrt(2 * g * (delta_y - delta_x))) {
            return Optional.empty();
        }

        return Optional.of(new Rotation2d(theta));
    }

    public double getAngle() {
        return -((shooterEncoder.getAbsolutePosition()) * 2 * Math.PI
                + Math.toRadians(ShooterConstants.kShooterEncoderOffset));
    }

    public void setGoal(double goalAngle) {
        double angle = getAngle();
        double kg = feedforward.calculate(angle, 0);

        goalAngle = Math.toRadians(goalAngle);

        double pidOutput = pidController.calculate(angle, goalAngle);

        double outputVolts = pidOutput + Math.cos(angle)
                * (goalAngle < ShooterConstants.kGravityLimit ? 0 : ShooterConstants.kTorqueShooterConstant);

        if (angle < ShooterConstants.kMinPosition)
            outputVolts = Math.max(kg, Math.min(2, outputVolts));
        if (angle > ShooterConstants.kMaxPosition)
            outputVolts = Math.max(-2, Math.min(kg, outputVolts));

        outputVolts = Math.max(-ShooterConstants.kMaxShooterPositionVolts,
                Math.min(ShooterConstants.kMaxShooterPositionVolts, outputVolts));

        shooterMotor.setVoltage(outputVolts);
    }

    public Command moveTo(double goalAngle) {
        return run(
                () -> setGoal(goalAngle));
    }

    public Command liftShooter() {
        double raiseVolts = ShooterConstants.kRaiseShooterVolts
                + Math.cos(getAngle()) * ShooterConstants.kTorqueShooterConstant;
        final double outputVolts = raiseVolts;
        raiseVolts = Math.max(-ShooterConstants.kMaxShooterPositionVolts,
                Math.min(ShooterConstants.kMaxShooterPositionVolts, raiseVolts));

        return new PrintCommand("lifting Shooter")
                .andThen(run(
                        () -> shooterMotor.setVoltage(ShooterConstants.kRaiseShooterVolts
                                + Math.cos(getAngle()) * ShooterConstants.kTorqueShooterConstant)))
                .until(
                        () -> (Math.toDegrees(getAngle()) > 0))
                .andThen(runOnce(
                        () -> System.out.println("At " + Math.toDegrees(getAngle()) + ", stopping now")))
                .andThen(stop());
    }

    public Command lowerShooter() {
        return run(
                () -> shooterMotor.setVoltage(ShooterConstants.klowerShooterVolts)).until(
                        () -> shooterMotor.getOutputCurrent() > ShooterConstants.kShooterCurrentLimit);
    }

    public Command stop() {
        return run(
                () -> {
                    shooterMotor.set(0);
                }).withName("stop");
    }
}
