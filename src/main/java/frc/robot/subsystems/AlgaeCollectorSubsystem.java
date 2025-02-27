// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeSubystemConstants;

public class AlgaeCollectorSubsystem extends SubsystemBase {
  private SparkFlex motor = new SparkFlex(AlgaeSubystemConstants.MOTOR_ID, MotorType.kBrushless);
  
    /** Creates a new CoralSubystem. */
    public AlgaeCollectorSubsystem() {
      SparkFlexConfig motorConfig = new SparkFlexConfig();
      motorConfig.idleMode(IdleMode.kBrake);
      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    public void in() {
      motor.set(-1);
    } 
  
    public void out() {
      motor.set(1);
    }
  
    public void stop() {
      motor.stopMotor();
    }
  }