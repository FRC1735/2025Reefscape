// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralSubystemConstants;
import frc.robot.sensors.DistanceSensor;

public class CoralSubystem extends SubsystemBase {
private SparkFlex leadMotor = new SparkFlex(CoralSubystemConstants. LEAD_MOTOR_ID, MotorType.kBrushless);
private SparkFlex followMotor = new SparkFlex(CoralSubystemConstants.FOLLOW_MOTOR_ID, MotorType.kBrushless); 
  // TODO - delete these and put in subsystems
  DistanceSensor distanceSensor0 = new DistanceSensor(0);
  DistanceSensor distanceSensor1 = new DistanceSensor(1);
  DistanceSensor distanceSensor2 = new DistanceSensor(2);

  /** Creates a new CoralSubystem. */
  public CoralSubystem() {
    SparkFlexConfig leadMotorConfig = new SparkFlexConfig();
    leadMotorConfig.idleMode(IdleMode.kBrake);
    leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkFlexConfig followMotorConfig = new SparkFlexConfig();
    followMotorConfig.idleMode(IdleMode.kBrake);
    followMotorConfig.follow(CoralSubystemConstants.LEAD_MOTOR_ID, false);
    followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        // TODO - remove and put in subsystems
    SmartDashboard.putNumber("Distance Sensor 0", distanceSensor0.getDistance());
    SmartDashboard.putNumber("Distance Sensor 1", distanceSensor1.getDistance());
    SmartDashboard.putNumber("Distance Sensor 2", distanceSensor2.getDistance());

  }

  public void shoot() {
    leadMotor.set(0.15);
  } 

  public void returnToFunnel() {
    leadMotor.set(-0.2);
  }

  public void stop() {
    leadMotor.stopMotor();
  }
  
  public void collect() {
    leadMotor.set(0.25);
  } 
}




