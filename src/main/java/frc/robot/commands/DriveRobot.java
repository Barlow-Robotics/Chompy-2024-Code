// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class DriveRobot extends Command {

    Drive driveSub;
    Joystick driverController;

    int ControllerXSpeedID;
    int ControllerYSpeedID;
    int ControllerRotID;
    boolean FieldRelative;

    double DeadBand = 0.08;
    
    public DriveRobot(
        Drive driveSub, 
        Joystick driverController, 
        int ControllerXSpeedID, 
        int ControllerYSpeedID, 
        int ControllerRotID,
        boolean FieldRelative) {

        this.driveSub = driveSub;
        this.driverController = driverController;
        this.ControllerXSpeedID = ControllerXSpeedID;
        this.ControllerYSpeedID = ControllerYSpeedID;
        this.ControllerRotID = ControllerRotID;
        this.FieldRelative = FieldRelative;

        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // double rawX = driverController.getRawAxis(this.ControllerXSpeedID);
        // double rawY = driverController.getRawAxis(this.ControllerYSpeedID);

        // Since the coordinate systems differ from the controller (x is left right and y is fwd back) 
        // and the chassis (positive X is forward, Positive Y is left), we use the controller X input as the drive Y input
        // and the controller Y input as the drive X input.

// wpk had to change signs for logitech controller. Not sure if radio master needs this
        double rawX = -driverController.getRawAxis(this.ControllerYSpeedID);  
        double rawY = -driverController.getRawAxis(this.ControllerXSpeedID);
        double rawRot = -driverController.getRawAxis(this.ControllerRotID); 

        double XSpeed = MathUtil.applyDeadband(rawX, DeadBand) * Constants.DriveConstants.MaxDriveableVelocity;
        double YSpeed = MathUtil.applyDeadband(rawY, DeadBand) * Constants.DriveConstants.MaxDriveableVelocity;
        double Rot = MathUtil.applyDeadband(rawRot, 2*DeadBand) * Constants.DriveConstants.MaxDriveableVelocity;

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
