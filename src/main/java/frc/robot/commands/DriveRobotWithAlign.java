//Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElectronicsIDs;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants.DriveConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class DriveRobotWithAlign extends Command {
    Vision visionSub;
    Drive driveSub;
    Joystick driverController;

    int ControllerXSpeedID;
    int ControllerYSpeedID;
    int ControllerRotID;
    boolean FieldRelative;
    private double leftVelocity;
    private double rightVelocity;
    private double adjustment;
    private int missedFrames = 0;

    private double error;



    Trigger autoAlignButton;
    //Trigger toggleTargetButton;

    double DeadBand = 0.08;
    double RotDeadBand = 0.01;

    public PIDController rotPid;
    public PIDController latPid;
    
    public String selectedTarget = "None";

    
    public DriveRobotWithAlign(
        Drive driveSub,
        Joystick driverController, 
        int ControllerXSpeedID, 
        int ControllerYSpeedID, 
        int ControllerRotID,
        boolean FieldRelative,
        Vision visionSub, 
        Trigger autoAlignButton
        ) {
       
        this.driveSub = driveSub;
        this.driverController = driverController;
        this.ControllerXSpeedID = ControllerXSpeedID;
        this.ControllerYSpeedID = ControllerYSpeedID;
        this.ControllerRotID = ControllerRotID;
        this.FieldRelative = FieldRelative;
        this.autoAlignButton = autoAlignButton;
        this.visionSub = visionSub;

    

        rotPid = new PIDController(
                DriveConstants.AutoAlignRotKP,
                DriveConstants.AutoAlignRotKI,
                DriveConstants.AutoAlignRotKD);

        latPid = new PIDController(
            DriveConstants.AutoAlignLatKP,
            DriveConstants.AutoAlignLatKI,
            DriveConstants.AutoAlignLatKD);
        
         addRequirements(driveSub);
    }

    @Override
    public void initialize() {
        rotPid.reset();
        latPid.reset();
        visionSub.chooseBestTarget();
    }

    //add key for auto align button

    @Override
    public void execute() {
        // Since the coordinate systems differ from the controller (x is left right and y is fwd back) 
        // and the chassis (positive X is forward, Positive Y is left), we use the controller X input as the drive Y input
        // and the conztroller Y input as the drive X input.
        boolean autoAlignEnabled = autoAlignButton.getAsBoolean();
        //boolean toggleTarget = toggleTargetButton.getAsBoolean();
        var alignYawControl = 0.0;
        var alignLatControl = 0.0;

        if (autoAlignEnabled) {
            var rotOffset = visionSub.getTargetRotOffSet();
            var latOffset = visionSub.getTargetLateralOffSet();
            if (rotOffset.isPresent()){
                alignYawControl = rotPid.calculate(-rotOffset.getAsDouble());
            }
            
            if (latOffset.isPresent()){
                alignLatControl = latPid.calculate(-latOffset.getAsDouble());
            }
        }

        double rawX;
        double rawY;
        double rawRot;
        
        if(DriverStation.getJoystickName(ElectronicsIDs.DriverControllerPort).equals("Logitech Extreme 3D")) {
            rawX = -driverController.getRawAxis(this.ControllerYSpeedID);  
            rawY = -driverController.getRawAxis(this.ControllerXSpeedID);
            rawRot = -driverController.getRawAxis(this.ControllerRotID); 
        } else {
            rawX = driverController.getRawAxis(this.ControllerYSpeedID);  
            rawY = -driverController.getRawAxis(this.ControllerXSpeedID);
            rawRot = -driverController.getRawAxis(this.ControllerRotID); 
        }
        if (autoAlignEnabled) {
            rawRot = alignYawControl;

            // TODO: The way this mix happens needs to depend on which target we are
            // aligning with.

            var heading = Math.toRadians(driveSub.getHeading());

            // TODO: check this math
            // Heading of 0 means lateral offset is +y
            rawY += Math.cos(-heading) * alignLatControl;
            // Heading of pi/2 means lateral offset is -x
            rawX += Math.sin(-heading) * alignLatControl;
        }

        double XSpeed = MathUtil.applyDeadband(rawX, DeadBand) * DriveConstants.MaxDriveableVelocity;
        double YSpeed = MathUtil.applyDeadband(rawY, DeadBand) * DriveConstants.MaxDriveableVelocity;
        double Rot = MathUtil.applyDeadband(rawRot, RotDeadBand);

        driveSub.drive(XSpeed, YSpeed, Rot, FieldRelative);

        /* LOGGING */

        Logger.recordOutput("Align/YawControl", alignYawControl);
        Logger.recordOutput("Align/LatControl", alignLatControl);

        Logger.recordOutput("Align/RawYawInput", rawRot);
        Logger.recordOutput("Align/RawXSpeed", rawX);
        Logger.recordOutput("Align/RawYSpeed", rawY);

        Logger.recordOutput("Align/YawInput", Rot);
        Logger.recordOutput("Align/XSpeed", YSpeed);
        Logger.recordOutput("Align/YSpeed", XSpeed);
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