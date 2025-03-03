// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  Servo servo;

  boolean DEBUG = true;

  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    this.servo = new Servo(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DEBUG) {
      SmartDashboard.putNumber("Climber servo angle", servo.getAngle());
      SmartDashboard.putNumber("Climber servo position", servo.getPosition());
    }
  }


}
