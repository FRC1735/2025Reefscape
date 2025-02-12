// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristSubystemConstants;

public class WristSubsystem extends SubsystemBase {
  // TODO - REVERT ID TO `WristSubystemConstants.MOTOR_ID`
  private SparkFlex motor = new SparkFlex(10, MotorType.kBrushless);
  
  private static final SparkFlexConfig motorConfig = new SparkFlexConfig();
  
  static {
    // TODO - can probably delete these?
    

    // Calculations required for driving motor conversion factors and feed forward
    final double kDrivingMotorFreeSpeedRps = 5676 / 60;
    final double kWheelDiameterMeters = 0.0762;
    final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    final double kDrivingMotorReduction = (45.0 * 22) / (14 * 15);
    final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    double turningFactor = 2 * Math.PI;
    double drivingVelocityFeedForward = 1 / kDriveWheelFreeSpeedRps;


    motorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(20);
    motorConfig.absoluteEncoder
      // Invert the turning encoder, since the output shaft rotates in the opposite
      // direction of the steering motor in the MAXSwerve Module.
      .inverted(true)
      .positionConversionFactor(turningFactor) // radians
      .velocityConversionFactor(turningFactor / 60.0); // radians per second
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      // These are example gains you may need to them for your own robot!
      .pid(1, 0, 0)
      .outputRange(-1, 1)
      // Enable PID wrap around for the turning motor. This will allow the PID
      // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
      // to 10 degrees will go through 0 rather than the other direction which is a
      // longer route.
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(0, turningFactor);
  }

  private boolean DEBUG = true;

  private SparkClosedLoopController closedLoopController = motor.getClosedLoopController();

  public WristSubsystem() {

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    if (DEBUG) {
      SmartDashboard.putData("Test Wrist Rotate .5", new InstantCommand(this::goToReefLevelZero, this));
      SmartDashboard.putData("Test Wrist Rotate .25",  new InstantCommand(this::goToReefLevelOne, this));
      SmartDashboard.putData("Test Wrist Stop", new InstantCommand(this::stop, this));
    }
  } 
  
  @Override
  public void periodic() {
    if (DEBUG) {
      SmartDashboard.putNumber("Wrist Encoder", motor.getAbsoluteEncoder().getPosition());
    }
  }

  public void goToReefLevelZero() {
    closedLoopController.setReference(0.5, ControlType.kPosition);
  }

  public void goToReefLevelOne() {
    closedLoopController.setReference(0.25, ControlType.kPosition);
  }
  
  public void up() {
    motor.set(-0.1);
  } 

  public void down() {
    motor.set(0.1);
  }

  public void stop() {
    motor.stopMotor();
  }
}

