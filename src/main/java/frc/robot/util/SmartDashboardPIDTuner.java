// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardPIDTuner {
    private SparkFlex sparkFlex;
    private SparkFlexConfig sparkFlexConfig;

    private double p, i, d;

    private String pLabel;
    private String iLabel;
    private String dLabel;

    private boolean debug;

    // this is assuming that the passed sparkFlexConfig will have everything setup except the closedLoop config
    public SmartDashboardPIDTuner(
        String subsystemName, SparkFlex sparkFlex, SparkFlexConfig sparkFlexConfig,
         double startP, double startI, double startD,
         double outputMin, double outputMax,
         FeedbackSensor feedbackSensor,
         boolean positionWrappingEnabled,
         boolean debug
    ) {
        this.sparkFlexConfig = sparkFlexConfig;
        this.sparkFlex = sparkFlex;

        this.p = startP;
        this.i = startI;
        this.d = startD;

        this.debug = debug;

        sparkFlexConfig
            .closedLoop
            .feedbackSensor(feedbackSensor)
            .pid(p, i, d)
            .outputRange(outputMin, outputMax)
            .positionWrappingEnabled(positionWrappingEnabled);
            
        sparkFlex.configure(sparkFlexConfig, ResetMode.kResetSafeParameters,  PersistMode.kPersistParameters);

        this.pLabel = subsystemName + " - P";
        this.iLabel = subsystemName + " - I";
        this.dLabel = subsystemName + " - D";

        SmartDashboard.putNumber(pLabel, p);
        SmartDashboard.putNumber(iLabel, i);
        SmartDashboard.putNumber(dLabel, d);
    }

    public void periodic() {
        if (debug) {
            boolean changed = false;
            if (SmartDashboard.getNumber(pLabel, p) != p) {
                changed = true;
                p = SmartDashboard.getNumber(pLabel, p);
            }
            if (SmartDashboard.getNumber(iLabel, i) != i) {
                changed = true;
                i = SmartDashboard.getNumber(iLabel, i);
            }
            if (SmartDashboard.getNumber(dLabel, d) != d) {
                changed = true;
                p = SmartDashboard.getNumber(dLabel, d);
            }

            if (changed) {
                sparkFlexConfig.closedLoop.pid(p, i, d);
                sparkFlex.configure(sparkFlexConfig, ResetMode.kResetSafeParameters,  PersistMode.kPersistParameters);
            }
        }
    }
}
