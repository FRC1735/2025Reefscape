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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristSubystemConstants;
import frc.robot.util.SmartDashboardPIDTuner;

public class WristSubsystem extends SubsystemBase {
  private SparkFlex motor = new SparkFlex(WristSubystemConstants.MOTOR_ID, MotorType.kBrushless);
  private SmartDashboardPIDTuner smartDashboardPIDTuner;
  private final boolean DEBUG = true;

    public WristSubsystem() {
      SparkFlexConfig motorConfig = new SparkFlexConfig();
      motorConfig.idleMode(IdleMode.kBrake);
      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

      this.smartDashboardPIDTuner = new SmartDashboardPIDTuner(getSubsystem(), motor, motorConfig, 0, 0, 0, 0, 1, FeedbackSensor.kAbsoluteEncoder, false, DEBUG);
    }
  
    @Override
    public void periodic() {
      smartDashboardPIDTuner.periodic();
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

