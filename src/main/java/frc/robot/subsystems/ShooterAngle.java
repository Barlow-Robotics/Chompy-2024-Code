// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectronicIDs;

public class ShooterAngle extends SubsystemBase {

    public enum ShooterPositionState {
        Speaker, Amp, IntakeFromSource, IntakeFromFloor, Trap
    }

    public ShooterPositionState shooterAngleState = ShooterPositionState.IntakeFromFloor;

    TalonFX angleMotor;

    public ShooterAngle() {
        angleMotor = new TalonFX(ElectronicIDs.AngleMotorID);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /** @param angle Desired angle in fraction of a rotation */ //May want to CHANGE this to degrees
    public void setAngle(double angle) {
        angleMotor.setPosition(angle);
    }

    public double getAngle() {
        return angleMotor.getPosition().getValue();
    }

}
