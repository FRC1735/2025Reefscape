// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.SharpIR;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralSubystemConstants;

public class CoralSubystem extends SubsystemBase {
  private SparkFlex leadMotor = new SparkFlex(CoralSubystemConstants.LEAD_MOTOR_ID, MotorType.kBrushless);
  private SparkFlex followMotor = new SparkFlex(CoralSubystemConstants.FOLLOW_MOTOR_ID, MotorType.kBrushless);

  private SharpIR topDistanceSensor = SharpIR.GP2Y0A41SK0F(1);
  private SharpIR bottomDistanceSensor = SharpIR.GP2Y0A41SK0F(2);


  final double SHOOT_SPEED = 0.15;
  final double REVERSE_SPEED= -0.2;
  final double LOAD_SPEED = 0.04;

  final double BOTTOM_MODIFIER = 0.03;
  final double BOTTOM_SHOOT_SPEED = SHOOT_SPEED + BOTTOM_MODIFIER;
  final double BOTTOM_REVERSE_SPEED = REVERSE_SPEED - BOTTOM_MODIFIER;
  final double BOTTOM_LOAD_SPEED = LOAD_SPEED + BOTTOM_MODIFIER;

  private final boolean DEBUG = false;

  /** Creates a new CoralSubystem. */
  public CoralSubystem() {
    SparkFlexConfig leadMotorConfig = new SparkFlexConfig();
    leadMotorConfig.idleMode(IdleMode.kBrake);
    leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkFlexConfig followMotorConfig = new SparkFlexConfig();
    followMotorConfig.idleMode(IdleMode.kBrake);
    //followMotorConfig.follow(CoralSubystemConstants.LEAD_MOTOR_ID, false);
    followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Coral - Safe for Elevator Movement", isSafeForElevator().getAsBoolean());
    SmartDashboard.putBoolean("Coral - is loaded?", isCoralLoaded().getAsBoolean());

    if (DEBUG) {
      SmartDashboard.putNumber("Coral - Top Distance Sensor", topDistanceSensor.getRangeCM());
      SmartDashboard.putNumber("Coral - Bottom Distance Sensor", bottomDistanceSensor.getRangeCM());
      SmartDashboard.putBoolean("Coral - is loaded?", isCoralLoaded().getAsBoolean());
    }
  }

  public Command shoot() {
    return this.run(() -> {
      leadMotor.set(SHOOT_SPEED);
      followMotor.set(BOTTOM_SHOOT_SPEED);
    });
  }

  public Command reverse() {
    return this.run(() -> {
      leadMotor.set(REVERSE_SPEED);
      followMotor.set(BOTTOM_REVERSE_SPEED);
    });
  }

  public Command stop() {
    return this.runOnce(() -> {
      leadMotor.stopMotor();
      followMotor.stopMotor();
    });
  }

  public Command load() {
    return this.run(() -> this.loadCoral());
  }

  // TODO - all elevator related commands should check this and not do anything if
  // coral isn't considered safe (ie it will block the elevator from moving)
  public BooleanSupplier isSafeForElevator() {
    return () -> 
      // no coral detected
      (topDistanceSensor.getRangeCM() >= 14)
      // coral loaded properly
      || isCoralLoaded().getAsBoolean();
  }

  public BooleanSupplier isCoralLoaded() {
    return () -> (topDistanceSensor.getRangeCM() >= 14 && bottomDistanceSensor.getRangeCM() <= 6);
  }

  public void loadCoral() {
    leadMotor.set(LOAD_SPEED);
    followMotor.set(BOTTOM_LOAD_SPEED);
  }

  public boolean coralDetected() {
    return topDistanceSensor.getRangeCM() <= 10.5;
  }

  public boolean coralInPosition() {
    return bottomDistanceSensor.getRangeCM() <= 7;
  }

  public void stopCollectin() {
    leadMotor.stopMotor();
    followMotor.stopMotor();
  }
}
