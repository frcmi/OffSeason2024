package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.variableShooterConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class VariableShooterSubsystem extends SubsystemBase {
    public final CANSparkMax shooterMotor = new CANSparkMax(variableShooterConstants.kVariableShooterMotorId, MotorType.kBrushless);
    private final DutyCycleEncoder shooterEncoder = new DutyCycleEncoder(variableShooterConstants.kVariableShooterEncoderId);
    private final ProfiledPIDController pidController = new ProfiledPIDController(variableShooterConstants.kP, variableShooterConstants.kI, variableShooterConstants.kD, new TrapezoidProfile.Constraints(variableShooterConstants.kMaxLiftVelocity, variableShooterConstants.kMaxLiftAcceleration));

    private final ArmFeedforward feedforward = new ArmFeedforward(variableShooterConstants.kS, variableShooterConstants.kV, variableShooterConstants.kA);
    
    public VariableShooterSubsystem() {
        shooterEncoder.setDistancePerRotation(1);
        shooterEncoder.setPositionOffset(variableShooterConstants.kShooterEncoderOffset / 360);
        shooterMotor.setIdleMode(IdleMode.kBrake);

        shooterMotor.setInverted(false);

        setDefaultCommand(stop());
    }

    public double getAngle() {
        return -((shooterEncoder.getAbsolutePosition()) * 2 * Math.PI + Math.toRadians(variableShooterConstants.kShooterEncoderOffset));
        }

    public void setGoal(double goalAngle) {
        double angle = getAngle();
        double kg = feedforward.calculate(angle, 0);

        goalAngle = Math.toRadians(goalAngle);

        double pidOutput = pidController.calculate(angle, goalAngle);

        double outputVolts = pidOutput + Math.cos(angle) * (goalAngle < variableShooterConstants.kGravityLimit ? 0 : variableShooterConstants.kTorqueShooterConstant);

        if (angle < variableShooterConstants.kMinPosition)
            outputVolts = Math.max(kg, Math.min(2, outputVolts));
        if (angle > variableShooterConstants.kMaxPosition)
            outputVolts = Math.max(-2, Math.min(kg, outputVolts));

        outputVolts = Math.max(-variableShooterConstants.kMaxShooterPositionVolts, Math.min(variableShooterConstants.kMaxShooterPositionVolts, outputVolts));

        shooterMotor.setVoltage(outputVolts);
    }

    public Command moveTo(double goalAngle) {
        return run(
            () -> setGoal(goalAngle));
    }

    public Command liftShooter() {
        double raiseVolts = variableShooterConstants.kRaiseShooterVolts + Math.cos(getAngle()) * variableShooterConstants.kTorqueShooterConstant;
        final double outputVolts = raiseVolts;
        raiseVolts = Math.max(-variableShooterConstants.kMaxShooterPositionVolts, Math.min(variableShooterConstants.kMaxShooterPositionVolts, raiseVolts));
        
        return new PrintCommand("lifting Shooter")
        .andThen(run(
            () -> shooterMotor.setVoltage(variableShooterConstants.kRaiseShooterVolts + Math.cos(getAngle()) * variableShooterConstants.kTorqueShooterConstant)))
        .until(
            () -> (Math.toDegrees(getAngle()) > 0))
        .andThen(runOnce(
            () -> System.out.println("At " + Math.toDegrees(getAngle()) +", stopping now")))
        .andThen(stop());
    }

    public Command lowerShooter() {
        return run(
            () -> shooterMotor.setVoltage(variableShooterConstants.klowerShooterVolts)
        ).until(
            () -> shooterMotor.getOutputCurrent() > variableShooterConstants.kShooterCurrentLimit
        );
    }

    public Command stop() {
        return run(
            () -> {
                shooterMotor.set(0);
            }
        ).withName("stop");
    }
}

