// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ClimberConstants;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// add logging later

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax leftClimberMotor = new CANSparkMax(ClimberConstants.kLeftClimberId, MotorType.kBrushless);
   private final CANSparkMax rightClimberMotor = new CANSparkMax(ClimberConstants.kRightClimberId, MotorType.kBrushless);

  public ClimberSubsystem() {
    leftClimberMotor.setIdleMode(IdleMode.kBrake);
  rightClimberMotor.setIdleMode(IdleMode.kBrake);
    setDefaultCommand(stop());

  }

  public Command up() {
    return run (
            () -> {leftClimberMotor.set(ClimberConstants.kClimberUp);
                rightClimberMotor.set(ClimberConstants.kClimberUp);
            }
    ).withName("up");
}

public Command down() { 
    return run (
            () -> {leftClimberMotor.set(ClimberConstants.kClimberDown);
                rightClimberMotor.set(ClimberConstants.kClimberDown);
            }
    ).withName("down");
}

    public Command stop() {
        return run(
            () -> {leftClimberMotor.set(0);
                rightClimberMotor.set(0);
                }
        ).withName("stop");
    }
}
