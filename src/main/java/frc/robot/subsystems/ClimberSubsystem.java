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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SmartDashboardPIDTuner;

public class ClimberSubsystem extends SubsystemBase {
  SparkFlex motor = new SparkFlex(Constants.ClimberConstants.MOTOR_ID, MotorType.kBrushless);
  DigitalInput lowerLimitSwitch = new DigitalInput(Constants.ClimberConstants.LOWER_LIMIT_ID);
  DigitalInput upperLimitSwitch = new DigitalInput(Constants.ClimberConstants.UPPER_LIMIT_ID);
  SmartDashboardPIDTuner smartDashboardPIDTuner;
  boolean DEBUG = true;
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);

    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .maxMotion
      .maxVelocity(0) // TODO - determine based on gear ratio
      .maxAcceleration(0) // TODO - determine based on gear ratio
      .allowedClosedLoopError(0.025);

      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

      // TODO - not sure about output min / max
      smartDashboardPIDTuner = new SmartDashboardPIDTuner("Climber", motor, motorConfig, 0.1, 0, 0, -1, 1, FeedbackSensor.kPrimaryEncoder, false, DEBUG);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DEBUG) {
      smartDashboardPIDTuner.periodic();
      SmartDashboard.putBoolean("Climber - upper limit", upperLimitSwitch.get());
      SmartDashboard.putBoolean("Climber - lower limit", lowerLimitSwitch.get());
    }
  }
}
