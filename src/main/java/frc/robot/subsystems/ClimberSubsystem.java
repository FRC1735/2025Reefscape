// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SmartDashboardPIDTuner;

public class ClimberSubsystem extends SubsystemBase {
  SparkFlex motor = new SparkFlex(Constants.ClimberConstants.MOTOR_ID, MotorType.kBrushless);
  DigitalInput lowerLimitSwitch = new DigitalInput(Constants.ClimberConstants.LOWER_LIMIT_ID);
  DigitalInput upperLimitSwitch = new DigitalInput(Constants.ClimberConstants.UPPER_LIMIT_ID);
  boolean DEBUG = false;
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kCoast); // 190 is using Coast as the wratchet provides a breaking effect
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DEBUG) {
      //SmartDashboard.putBoolean("Climber - upper limit", upperLimitSwitch.get());
      //SmartDashboard.putBoolean("Climber - lower limit", lowerLimitSwitch.get());
      SmartDashboard.putNumber("Climber - encoder", motor.getEncoder().getPosition());
    }
  }

  double CLIMB_DEPLOY_TICKS = -6;
  public Command deploy() {
    return Commands.run(() -> {
      if (motor.getEncoder().getPosition() < CLIMB_DEPLOY_TICKS) {
        motor.stopMotor();
      } else {
        motor.set(-0.5);
      }
    }, this);
  }

  // todo delete
  // TODO - determine encoder ticks to stop at and appropriate speed
  public Command climb() {
    return Commands.run(() -> {
      if (motor.getEncoder().getPosition() < STOP_CLIMB_ENCODER_TICKS) {
        motor.stopMotor();
      } else {
        motor.set(-1);
      }

    });
  }

  double STOP_CLIMB_ENCODER_TICKS = -142; 
  public Command climb2() {
    return Commands.runEnd(
      () -> motor.set(-1), 
      () -> motor.stopMotor())
      .until(
      () -> motor.getEncoder().getPosition() < STOP_CLIMB_ENCODER_TICKS);
  }

  public Command testPositive() {
    return Commands.run(() -> motor.set(0.2), this);
  }

  public Command testNegative() {
    return Commands.run(() -> motor.set(-0.2), this);
  }

  public Command testStop() {
    return Commands.run(() -> motor.stopMotor(), this);
  }
}
