// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.DriveConstants;

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
    public static double rawX;
    public static double rawY;
    public static double rawRot;
    public static double SpeedX;
    public static double SpeedY;
    public static double SpeedRot;
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
        // Since the coordinate systems differ from the controller (x is left right and y is fwd back) 
        // and the chassis (positive X is forward, Positive Y is left), we use the controller X input as the drive Y input
        // and the controller Y input as the drive X input.
        
        rawX = -driverController.getRawAxis(this.ControllerYSpeedID);  
        rawY = -driverController.getRawAxis(this.ControllerXSpeedID);
        rawRot = -driverController.getRawAxis(this.ControllerRotID); 

        SpeedX = MathUtil.applyDeadband(rawX, DeadBand) * DriveConstants.MaxDriveableVelocity;
        SpeedY = MathUtil.applyDeadband(rawY, DeadBand) * DriveConstants.MaxDriveableVelocity;
        SpeedRot = MathUtil.applyDeadband(rawRot, 2*DeadBand) * DriveConstants.MaxDriveableVelocity;

        driveSub.drive(SpeedX, SpeedY, SpeedRot, FieldRelative);      
        // driveSub.testDrive(()->driveSub.getX(), ()->driveSub.getY(), ()->driveSub.getRot(), true);
    }

    @Override
    public void end(boolean interrupted) {
        driveSub.drive(0, 0, 0, true);
        // driveSub.testDrive(()->0.0, ()->0.0, ()->0.0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
