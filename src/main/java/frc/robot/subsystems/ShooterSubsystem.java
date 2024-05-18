package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;

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
            outputVolts = MathUtil.clamp(outputVolts, -kg, 2);
        if (angle > ShooterConstants.kMaxPosition)
            outputVolts = MathUtil.clamp(outputVolts, -2, kg);

        outputVolts = MathUtil.clamp(outputVolts, -ShooterConstants.kMaxShooterPositionVolts,
                ShooterConstants.kMaxShooterPositionVolts);

        shooterMotor.setVoltage(outputVolts);
    }

    public Command moveTo(double goalAngle) {
        return run(() -> setGoal(goalAngle));
    }

    public void doStop() {
        shooterMotor.set(0);
    }

    public Command stop() {
        return run(this::doStop).withName("stop");
    }
}
