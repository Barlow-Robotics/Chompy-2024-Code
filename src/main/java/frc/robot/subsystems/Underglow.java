// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Underglow extends SubsystemBase {
    /** Creates a new UnderGlow. */
    public boolean LEDHumanSource = false;
    public boolean LEDHumanFloor = false;
    SerialPort port = null;

    int currentMode = 1;

    public Underglow() {
        try {
            port = new SerialPort(9600, Constants.UnderGlowConstants.Port);
        } catch (Exception ex) {

        }
    }

    @Override
    public void periodic() {
        int desiredMode = 0;
        
        Alliance alliance = DriverStation.getAlliance().get();

        if (alliance == Alliance.Blue) {
            desiredMode += Constants.UnderGlowConstants.BlueAlliance;
        } else if (alliance == Alliance.Red) {
            desiredMode += Constants.UnderGlowConstants.RedAlliance;
        } else if (LEDHumanSource == true) {
            desiredMode += Constants.UnderGlowConstants.RobotSource;
        } if (LEDHumanFloor == true)
        {
            desiredMode += Constants.UnderGlowConstants.RobotFloorSource;
        }
        

        if (currentMode != desiredMode && port != null) {
            try {
                port.write(new byte[] { (byte) desiredMode }, 1);
            } catch (Exception ex) {

            }
            currentMode = desiredMode;
        }
    }
}