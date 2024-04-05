// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TuneableParameter {
    final double startingNum;
    final double maxValue;
    final double minValue;
    final boolean editable;
    final String key;

    public TuneableParameter(double startingNum, double maxValue, double minValue, boolean editable, String key) {
        this.startingNum = startingNum;
        this.maxValue = maxValue;
        this.minValue = minValue;
        this.editable = editable;
        this.key = key;
        SmartDashboard.putNumber(key, startingNum);
    }

    public double get() {
        double currentValue = SmartDashboard.getNumber(key, startingNum);
        if (!editable) {
            currentValue = startingNum;
        } else {
            if (currentValue < minValue) {
                currentValue = minValue;
            } else if (currentValue > maxValue) {
                currentValue = maxValue;
            }
        }
        return currentValue;
    }
}
