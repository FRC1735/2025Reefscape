// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristSubystemConstants;

public class WristSubsystem extends SubsystemBase {
  private SparkFlex motor = new SparkFlex(WristSubystemConstants.MOTOR_ID, MotorType.kBrushless);
  private SparkClosedLoopController closedLoopController;

  boolean DEBUG = true;

  public WristSubsystem() {
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);

    motorConfig
      .absoluteEncoder
      .inverted(true);

    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(1, 0, 0)
      .outputRange(-1, 1)
      .positionWrappingEnabled(false);

    motorConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(0.2341)
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(0.5532); // TODO

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  

    closedLoopController = motor.getClosedLoopController();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DEBUG) {
      SmartDashboard.putNumber("Wrist Encoder", motor.getAbsoluteEncoder().getPosition());
    }
  }

  public void testPositionControl() {
      closedLoopController.setReference(0.4, ControlType.kPosition);
  }

  /*
  public void up() {
    motor.set(-1);
  } 

  public void down() {
    motor.set(1);
  }
    */

  public void stop() {
    motor.stopMotor();
  }
}

