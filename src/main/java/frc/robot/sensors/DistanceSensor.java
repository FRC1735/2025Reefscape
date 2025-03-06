// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Sharp GP2Y0A41SK0F Analog Distance Sensor 4-30cm
 * 
 * https://www.pololu.com/product/2464
 */
public class DistanceSensor {
    AnalogInput analogInput;

    public DistanceSensor(int port) {
        analogInput = new AnalogInput(port);
    }

    public double getDistance() {
        double voltage = Math.max(analogInput.getVoltage(), 0.00001);
        double distance = 12.84 * Math.pow(voltage, -0.9824);

        double result = Math.max(Math.min(distance, 35.0), 4.5);

        return result;
    }
}