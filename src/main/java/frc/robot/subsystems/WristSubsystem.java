// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristSubystemConstants;

public class WristSubsystem extends SubsystemBase {

  // TODO - REVERT ID TO `WristSubystemConstants.MOTOR_ID`
  private SparkFlex motor = new SparkFlex(10, MotorType.kBrushless);
  private SparkClosedLoopController pidController = motor.getClosedLoopController();
  
  private boolean DEBUG = true;

  public WristSubsystem() {
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);
  
    AbsoluteEncoderConfig absoluteEncoderConfig = new AbsoluteEncoderConfig();
    absoluteEncoderConfig.setSparkMaxDataPortConfig();

    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();

    closedLoopConfig
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(0.02)
      .i(0)
      .d(0)
      .outputRange(-1, 1);

    motorConfig.apply(absoluteEncoderConfig);
    motorConfig.apply(closedLoopConfig);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (DEBUG) {
      SmartDashboard.putData("Test Wrist Rotate", new InstantCommand(this::goToReefLevelZero, this));
    }
  } 
  
  @Override
  public void periodic() {
    if (DEBUG) {
      SmartDashboard.putNumber("Wrist Encoder", motor.getAbsoluteEncoder().getPosition());
    }
  }

  public void goToReefLevelZero() {
    pidController.setReference(0.5, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
  
  public void up() {
    motor.set(-1);
  } 

  public void down() {
    motor.set(1);
  }

  public void stop() {
    motor.stopMotor();
  }
}

