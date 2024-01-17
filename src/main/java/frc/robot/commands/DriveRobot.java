// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class DriveRobot extends Command {

    Drive driveSub;
    PS4Controller driverController;

    int ControllerXSpeedID;
    int ControllerYSpeedID;
    int ControllerRotID;

    boolean FieldRelative;

    double DeadBand = 0.08;
    double MaxVelocity = 3.6; //meters per second //value is chosen, not calculated
    double MaxRotVelocity = 2.0; //radians per second
    int MaxRPM = 5676;
    
    public DriveRobot(
        Drive driveSub, 
        PS4Controller driverController, 
        int ControllerXSpeedID, 
        int ControllerYSpeedID, 
        int ControllerRotID,
        boolean FieldRelative) {

        this.driveSub = driveSub;
        this.driverController = driverController;
        this.ControllerXSpeedID = ControllerXSpeedID;
        this.ControllerYSpeedID = ControllerYSpeedID;
        this.FieldRelative = FieldRelative;

        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double rawX = -driverController.getLeftY() ;
        double rawY = -driverController.getLeftX() ;
        double rawRot = -driverController.getRightX() ; 

        double XSpeed = MathUtil.applyDeadband(rawX, DeadBand) * MaxVelocity;
        double YSpeed = MathUtil.applyDeadband(rawY, DeadBand) * MaxVelocity;
        double Rot = MathUtil.applyDeadband(rawRot, 2*DeadBand) * MaxRotVelocity;

        driveSub.drive(XSpeed, YSpeed, Rot, FieldRelative);

        /* LOGGING */
        Logger.recordOutput("Raw Yaw Input", rawRot);
        Logger.recordOutput("Raw XSpeed", rawX);
        Logger.recordOutput("Raw YSpeed", rawY);

        Logger.recordOutput("Yaw Input", Rot);
        Logger.recordOutput("XSpeed", YSpeed);
        Logger.recordOutput("YSpeed", XSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSub.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
