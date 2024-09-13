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

/*
 * Handles shooter arm motion and note firing
 */
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

        setDefaultCommand(stopArm());
    }

    /*
     * Retrieve the angle at which the shooter is currently aiming
     */
    public double getAngle() {
        return -((shooterEncoder.getAbsolutePosition()) * 2 * Math.PI
                + Math.toRadians(ShooterConstants.kShooterEncoderOffset));
    }

    /*
     * Set and update the angle setpoint of the arm motor - must be called
     * periodically
     */
    public void setArmGoal(double goalAngle) {
        double angle = getAngle();
        double kg = feedforward.calculate(angle, 0);

        double goalRadians = Math.toRadians(goalAngle);
        double pidOutput = pidController.calculate(angle, goalRadians);

        double outputVolts = pidOutput + Math.cos(angle)
                * (goalRadians < ShooterConstants.kGravityLimit ? 0 : ShooterConstants.kTorqueShooterConstant);

        // clamp by feedforward
        if (angle < ShooterConstants.kMinPosition)
            outputVolts = MathUtil.clamp(outputVolts, -kg, 2);
        if (angle > ShooterConstants.kMaxPosition)
            outputVolts = MathUtil.clamp(outputVolts, -2, kg);

        // clamp by global max voltage
        outputVolts = MathUtil.clamp(outputVolts, -ShooterConstants.kMaxShooterPositionVolts,
                ShooterConstants.kMaxShooterPositionVolts);

        // set setpoint
        shooterMotor.setVoltage(outputVolts);
    }

    /*
     * Arm move command - this is for general robot control
     */
    public Command moveArmTo(double goalAngle) {
        return run(() -> setArmGoal(goalAngle)).withName("moveArmTo").finallyDo(this::doStop);
    }

    public void doStop() {
        shooterMotor.set(0);
    }

    public Command stopArm() {
        return run(this::doStop).withName("stopArm");
    }
}
