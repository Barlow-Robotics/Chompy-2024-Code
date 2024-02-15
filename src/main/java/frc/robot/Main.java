// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.SwerveModule;

public final class Main {
    private Main() {

        System.out.println("\n\n\n\n\n\n\n///////////////////////////////////////////////\n" + SwerveModule.MaxVelocityPerSecond + "\n///////////////////////////////////////////////\n\n\n\n\n\n\n");
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
