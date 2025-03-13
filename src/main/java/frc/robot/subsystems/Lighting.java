// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.KeyboardController.Elevator;

public class Lighting extends SubsystemBase {
  //private AddressableLED ledLeft;
  //private AddressableLEDBuffer bufferLeft;

  private AddressableLED ledRight;
  private AddressableLEDBuffer bufferRight;

  int selectedR = 0;
  int selectedG = 0;
  int selectedB = 0;

  private int LED_COUNT = 60; // 60

  /** Creates a new Lighting. */
  public Lighting() {
    //ledLeft = new AddressableLED(0);
    ledRight = new AddressableLED(1);
    
    //bufferLeft = new AddressableLEDBuffer(LED_COUNT);
    bufferRight = new AddressableLEDBuffer(LED_COUNT);
    //ledLeft.setLength(bufferLeft.getLength());
    ledRight.setLength(bufferRight.getLength());

    //ledLeft.setData(bufferLeft);
    ledRight.setData(bufferRight);
    //ledLeft.start();
    ledRight.start();

    setColor(255, 255, 0);
  }

  double lastMatchTime = 30;
  boolean flashed = false;

  @Override
  public void periodic() {
    double matchTime = DriverStation.getMatchTime();

    if (!DriverStation.isAutonomous() && (matchTime < 30 && matchTime != -1) && DriverStation.isEnabled()) {
      if (lastMatchTime - matchTime > .5) {
        flashed = !flashed;
        lastMatchTime = matchTime;
      }
     if (flashed) {
        //System.out.println("red");
        setColor(255, 0, 0);
      } else {
        setColor(selectedR, selectedG, selectedB);
      }
    } else if (!SmartDashboard.getBoolean("Coral - Safe for Elevator Movement", false)) {
      red();
    } else if (SmartDashboard.getBoolean("Coral - is loaded?", false)) {
      coral();
    } else if (SmartDashboard.getBoolean("Algae Held", false)) {
      algae();
    } else {
      green();
    }
  }

  public void on() {
    setColor(0, 255, 0);
    //ledLeft.start();
    //ledRight.start();
  }

  public void off() {
    setColor(0, 0, 0);
    //ledLeft.stop();
    ledRight.stop();
  }

  public void coral() {
    setColor(252, 251, 244);
  }

  public void algae() {
    setColor(0, 0, 255);
  }

  public void blank() {
    setColor(0, 0, 0);
  }

  public void green() {
    setColor(0, 255, 0);
  }

  public void red() {
    setColor(255, 0, 0);
  }

  public void purple() {
    setColor(128 , 0, 128);
  }

  public void yellow() {
    setColor(255,255,0);
  }

  public void setColor(int r, int g, int b) {
    if (!(r == 255 && g == 0 && b == 0)) {
      selectedR = r;
      selectedB = b;
      selectedG = g;
    }

    for (int i = 0; i < bufferRight.getLength(); i++) {
      //bufferLeft.setRGB(i, r, g, b);
      bufferRight.setRGB(i, r, g, b);
    }
    //ledLeft.setData(bufferLeft);
    ledRight.setData(bufferRight);
  }
}
