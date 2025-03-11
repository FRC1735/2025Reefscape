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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSubystemConstants;
import frc.robot.utils.SmartDashboardPIDTuner;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkFlex leadMotor = new SparkFlex(ElevatorSubystemConstants.LEAD_MOTOR_ID, MotorType.kBrushless);
  private SparkFlex followMotor = new SparkFlex(ElevatorSubystemConstants.FOLLOW_MOTOR_ID, MotorType.kBrushless); 

  boolean DEBUG = true;
  private SmartDashboardPIDTuner smartDashboardPIDTuner;
  private SparkClosedLoopController closedLoopController;
  private double initialZeroPoint = leadMotor.getExternalEncoder().getPosition();

  // TODO - offset all of these w/ initial zero
  // setpoints

  // General
  private final double STORAGE = 0.2;

  // Algae specific
  private final double ALGAE_BARGE = 7.19;
  private final double ALGAE_L2 = 2.8;
  private final double ALGAE_L3 = 4.69;
  private final double ALGAE_PROCESSOR = STORAGE;
  private final double ALGAE_HELD = STORAGE;
  private final double ALGAE_GROUND = STORAGE;

  // Coral specific
  private final double CORAL_L1 = 1.1;
  private final double CORAL_L2 = 2.41;
  private final double CORAL_L3 = 4.27;
  private final double CORAL_L4 = 7.19;


  public ElevatorSubsystem() {
    SparkFlexConfig leadMotorConfig = new SparkFlexConfig();
    leadMotorConfig.idleMode(IdleMode.kBrake);
    leadMotorConfig
      .externalEncoder
      .countsPerRevolution(8192)
      .inverted(false)
      .positionConversionFactor(1);

    leadMotorConfig.softLimit
      .forwardSoftLimitEnabled(true) 
      .forwardSoftLimit(8.2)
      .reverseSoftLimitEnabled(true) // TODO - figure out how to set this based on where the enocder starts (it prob wont be 0)
      .reverseSoftLimit(STORAGE); // ????

    leadMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .outputRange(-1, 1)
      .positionWrappingEnabled(false)
      .maxMotion
      .maxVelocity(271.36)
      .maxAcceleration(1084)
      .allowedClosedLoopError(0.025);
    
    leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkFlexConfig followMotorConfig = new SparkFlexConfig();
    followMotorConfig.follow(ElevatorSubystemConstants.LEAD_MOTOR_ID, false);
    followMotorConfig.idleMode(IdleMode.kBrake);
    followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = leadMotor.getClosedLoopController();

    smartDashboardPIDTuner = new SmartDashboardPIDTuner("Elevator", leadMotor, leadMotorConfig, 0.8, 0, 0, -1, 1, FeedbackSensor.kAlternateOrExternalEncoder, false, DEBUG);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DEBUG) {
      SmartDashboard.putNumber("Elevator Encoder", leadMotor.getExternalEncoder().getPosition());
      smartDashboardPIDTuner.periodic();
      SmartDashboard.putNumber("Elevator Voltage", leadMotor.getAppliedOutput());
      SmartDashboard.putNumber("Elevator intial zero point", initialZeroPoint);
    }
  }

  // TODO - don't really like this, should probably be going towards the top and maintain that position
  public Command up() {
    return this.runOnce(() -> leadMotor.set(0.4));
  } 

  // TODO - don't really like this, should probably be going towards the bottom and maintain that position
  public Command down() {
    return this.runOnce(() -> leadMotor.set(-0.4));
  }

  public Command stop() {
    return this.runOnce(() -> leadMotor.stopMotor());
  }

  public Command algaeL2() {
    return this.runOnce(() -> closedLoopController.setReference(ALGAE_L2, ControlType.kMAXMotionPositionControl));
  }

  public Command algaeL3() {
    return this.runOnce(() -> closedLoopController.setReference(ALGAE_L3, ControlType.kMAXMotionPositionControl));
  }

  public Command algaeProcessor() {
    return storage();
  }

  public Command algaeGround() {
    return storage();
  }

  public Command algaeHeld() {
    return storage();
  }

  public Command algaeBarge() {
    return this.runOnce(() -> closedLoopController.setReference(ALGAE_BARGE, ControlType.kMAXMotionPositionControl));
  }

  public Command coralL1() {
    return this.runOnce(() -> closedLoopController.setReference(CORAL_L1, ControlType.kMAXMotionPositionControl));
  }

  public Command coralL2() {
    return this.runOnce(() -> closedLoopController.setReference(CORAL_L2, ControlType.kMAXMotionPositionControl));
  }

  public Command coralL3() {
    return this.runOnce(() -> closedLoopController.setReference(CORAL_L3, ControlType.kMAXMotionPositionControl));
  }

  public Command coralL4() {
    return this.runOnce(() -> closedLoopController.setReference(CORAL_L4, ControlType.kMAXMotionPositionControl));
  }

  public Command storage() {
    return this.runOnce(() -> closedLoopController.setReference(STORAGE, ControlType.kMAXMotionPositionControl));
  }
}