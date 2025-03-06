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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSubystemConstants;
import frc.robot.utils.SmartDashboardPIDTuner;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkFlex leadMotor = new SparkFlex(ElevatorSubystemConstants.LEAD_MOTOR_ID, MotorType.kBrushless);
  private SparkFlex followMotor = new SparkFlex(ElevatorSubystemConstants.FOLLOW_MOTOR_ID, MotorType.kBrushless); 

  boolean DEBUG = true;
  private SmartDashboardPIDTuner smartDashboardPIDTuner;
  private SparkClosedLoopController closedLoopController;

  public ElevatorSubsystem() {
    SparkFlexConfig leadMotorConfig = new SparkFlexConfig();
    leadMotorConfig.idleMode(IdleMode.kBrake);
    leadMotorConfig
      .externalEncoder
      .countsPerRevolution(8192)
      .inverted(false)
      .positionConversionFactor(1);

      // TODO These numbers were meant for conversion factor of 100 not 1!!
    leadMotorConfig.softLimit
      .forwardSoftLimitEnabled(false) // TODO - up doesn't work when this is true???
      .forwardSoftLimit(730) // TODO - verify this is working again, did not save code where it was "true"
      .reverseSoftLimitEnabled(false) // TODO - figure out how to set this based on where the enocder starts (it prob wont be 0)
      .reverseSoftLimit(100);

    leadMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .outputRange(-1, 1)
      .positionWrappingEnabled(false)
      .maxMotion
      .maxVelocity(271.36)
      .maxAcceleration(1084) // TODO - this is form teh wrist, need to determine more accurate value
      .allowedClosedLoopError(0.0025);
    
    leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkFlexConfig followMotorConfig = new SparkFlexConfig();
    followMotorConfig.follow(ElevatorSubystemConstants.LEAD_MOTOR_ID, false);
    followMotorConfig.idleMode(IdleMode.kBrake);
    followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = leadMotor.getClosedLoopController();

    smartDashboardPIDTuner = new SmartDashboardPIDTuner("Elevator", leadMotor, leadMotorConfig, 1, 0, 0, -1, 1, FeedbackSensor.kAlternateOrExternalEncoder, false, DEBUG);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DEBUG) {
      SmartDashboard.putNumber("Elevator Encoder", leadMotor.getExternalEncoder().getPosition());
      smartDashboardPIDTuner.periodic();
      SmartDashboard.putNumber("Elevator Voltage", leadMotor.getAppliedOutput());
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

  public void algaeL3() {
    closedLoopController.setReference(2.2, ControlType.kMAXMotionPositionControl);
  }

  public void algaeBarge() {
    closedLoopController.setReference(2.6, ControlType.kMAXMotionPositionControl);
  }


}