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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  SparkFlex motor = new SparkFlex(Constants.ClimberConstants.MOTOR_ID, MotorType.kBrushless);
  DigitalInput lowerLimitSwitch = new DigitalInput(Constants.ClimberConstants.LOWER_LIMIT_ID);
  DigitalInput upperLimitSwitch = new DigitalInput(Constants.ClimberConstants.UPPER_LIMIT_ID);

  boolean DEBUG = true;

  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DEBUG) {
      SmartDashboard.putBoolean("Climber - upper limit", upperLimitSwitch.get());
      SmartDashboard.putBoolean("Climber - lower limit", lowerLimitSwitch.get());
    }
  }
}
