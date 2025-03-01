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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSubystemConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkFlex leadMotor = new SparkFlex(ElevatorSubystemConstants.LEAD_MOTOR_ID, MotorType.kBrushless);
  private SparkFlex followMotor = new SparkFlex(ElevatorSubystemConstants.FOLLOW_MOTOR_ID, MotorType.kBrushless); 

  boolean DEBUG = true;

  public ElevatorSubsystem() {
    SparkFlexConfig leadMotorConfig = new SparkFlexConfig();
    leadMotorConfig.idleMode(IdleMode.kBrake);
    leadMotorConfig
      .externalEncoder
      .countsPerRevolution(8192)
      .inverted(false)
      .positionConversionFactor(100);

    leadMotorConfig.softLimit
      .forwardSoftLimitEnabled(false)
      .forwardSoftLimit(730) //
      .reverseSoftLimitEnabled(false) // TODO - figure out how to set this based on where the enocder starts (it prob wont be 0)
      .reverseSoftLimit(100);

    leadMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
    
    leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


    SparkFlexConfig followMotorConfig = new SparkFlexConfig();
    followMotorConfig.follow(ElevatorSubystemConstants.LEAD_MOTOR_ID, false);
    followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DEBUG) {
     SmartDashboard.putNumber("Elevator Encoder", leadMotor.getExternalEncoder().getPosition());
    }
  }

  public void up() {
    leadMotor.set(.4);
  } 

  public void down() {
    leadMotor.set(-0.4);
  }

  public void stop() {
    leadMotor.stopMotor();
  }
}