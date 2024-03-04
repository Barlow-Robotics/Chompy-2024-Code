// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.DriveConstants;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class DriveRobot extends Command {

    Drive driveSub;

    boolean FieldRelative;
    
    double DeadBand = 0.1;
    
    private double maxVelocityMultiplier = 1;

    Supplier<Double> xInput;
    Supplier<Double> yInput;
    Supplier<Double> rotInput;
    Supplier<Double> multiplierInput;
        
    public DriveRobot(
        Drive driveSub,
        Supplier<Double> x, 
        Supplier<Double> y, 
        Supplier<Double> rot, 
        Supplier<Double> multiplier,
        boolean FieldRelative) {

        this.driveSub = driveSub;
        this.xInput = x;
        this.yInput = y;
        this.rotInput = rot;
        this.multiplierInput = multiplier;
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

        // Converts from old range (1 to -1) to desired range (1 to 0.4, 4.5 m/s to 1.8 m/s)
        maxVelocityMultiplier = (((multiplierInput.get() + 1) * (1 - 0.4)) / 2) + 0.4;

        double speedX = MathUtil.applyDeadband(yInput.get(), DeadBand) * (DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier);
        double speedY = MathUtil.applyDeadband(xInput.get(), DeadBand) * (DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier);
        double speedRot = MathUtil.applyDeadband(rotInput.get(), 2*DeadBand) * DriveConstants.MaxAngularRadiansPerSecond;

        driveSub.drive(speedX, speedY, speedRot, FieldRelative);   

        Logger.recordOutput("Drive/YawInput", speedRot);
        Logger.recordOutput("Drive/XSpeed", speedY);
        Logger.recordOutput("Drive/YSpeed", speedX);
        Logger.recordOutput("Drive/Multipier", maxVelocityMultiplier);
    }

    // rride
    public void end(boolean interrupted) {
        driveSub.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
