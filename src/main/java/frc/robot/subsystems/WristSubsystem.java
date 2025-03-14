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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristSubystemConstants;
import frc.robot.utils.SmartDashboardPIDTuner;

public class WristSubsystem extends SubsystemBase {
  private SparkFlex motor = new SparkFlex(WristSubystemConstants.MOTOR_ID, MotorType.kBrushless);
  private SparkClosedLoopController closedLoopController;

  boolean DEBUG = false;
  private SmartDashboardPIDTuner smartDashboardPIDTuner;

  private double TOP_LIMIT = 0.5532;
  private double BOTTOM_LIMIT = 0.160;

  private double setpoint = 0;

  public WristSubsystem() {
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);

    motorConfig
      .absoluteEncoder
      .inverted(true);

    motorConfig
      .closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .outputRange(-1, 1)
      .positionWrappingEnabled(false)
      .maxMotion
      .maxVelocity(54.272)
      .maxAcceleration(216)
      .allowedClosedLoopError(0.025);

    motorConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(0.5532)
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(0.1678); // TODO

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    closedLoopController = motor.getClosedLoopController();

    smartDashboardPIDTuner = new SmartDashboardPIDTuner(
      "Wrist", motor, motorConfig, 0.5, 0, 0, -1, 1, FeedbackSensor.kAbsoluteEncoder, false, DEBUG);
        
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DEBUG) {
      SmartDashboard.putNumber("Wrist Encoder", motor.getAbsoluteEncoder().getPosition());
      smartDashboardPIDTuner.periodic();
      SmartDashboard.putNumber("Wrist Output", motor.getAppliedOutput());
    }
  }

  public Command algaeL2() {
    return this.runOnce(() -> closedLoopController.setReference(0.2351, ControlType.kMAXMotionPositionControl));
  }


  public Command algaeL3() {
    return this.runOnce(() -> closedLoopController.setReference(0.2351, ControlType.kMAXMotionPositionControl));
  }

  public Command algaeProcessor() {
    return this.runOnce(() -> closedLoopController.setReference(0.161, ControlType.kMAXMotionPositionControl));
  }

  public Command algaeGround() {
    return this.runOnce(() -> closedLoopController.setReference(0.161, ControlType.kMAXMotionPositionControl));
  }

  public Command algaeStorage() {
    return this.runOnce(() -> closedLoopController.setReference(0.5805, ControlType.kMAXMotionPositionControl));
  }

  public Command algaeHeld() {
    return this.runOnce(() -> closedLoopController.setReference(0.5287, ControlType.kMAXMotionPositionControl));
  }

  public Command algaeBarge() {
    return algaeHeld();
  }


  public Command up() {
    return this.run(() -> {
      double nextPosition = motor.getAbsoluteEncoder().getPosition() + 0.05;
      closedLoopController.setReference(nextPosition, ControlType.kMAXMotionPositionControl);
    });
  }

  public Command down() {
    return this.run(() -> {
      double nextPosition = motor.getAbsoluteEncoder().getPosition() - 0.05;
      closedLoopController.setReference(nextPosition, ControlType.kMAXMotionPositionControl);
    });
  }

  // This doesn't work the way I want it to
  public void maintainEncoderPosition() {
    //closedLoopController.setReference(motor.getAbsoluteEncoder().getPosition(), ControlType.kMAXMotionPositionControl);
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

