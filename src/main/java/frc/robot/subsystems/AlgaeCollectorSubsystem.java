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

import edu.wpi.first.wpilibj.SharpIR;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeSubystemConstants;

public class AlgaeCollectorSubsystem extends SubsystemBase {
  private SparkFlex motor = new SparkFlex(AlgaeSubystemConstants.MOTOR_ID, MotorType.kBrushless);

  private final boolean DEBUG = true;

  SharpIR distanceSensor = SharpIR.GP2Y0A41SK0F(0);

  public AlgaeCollectorSubsystem() {
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Algae Held", algaePresent());
    if (DEBUG) {
      SmartDashboard.putNumber("Algae Collector - Distance Sensor", distanceSensor.getRangeCM());
    }
  }

  private boolean algaePresent() {
    return distanceSensor.getRangeCM() < 6;
  }

  public Command in() {
    return this.run(() -> {
      if (algaePresent()) {
        motor.set(0);
      } else {
        motor.set(-1);
      }
    });
  }


  public Command out() {
    return this.run(() -> motor.set(1));
  }

  public Command stop() {
    return this.runOnce(() -> motor.stopMotor());
  }

  public boolean isAlgaeHeld() {
    return distanceSensor.getRangeCM() < 7;
  }

  public boolean isAlgaeCloseEnoughToGrab() {
    return distanceSensor.getRangeCM() < 15;
  }

  public void collect() {
    motor.set(-1);
  }

  public void stopCollecting() {
    motor.stopMotor();
  }
}