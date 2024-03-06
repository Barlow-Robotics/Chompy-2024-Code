//Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import java.util.OptionalInt;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class DriveRobotWithNoteAlign extends Command {

    Drive driveSub;
    Vision visionSub;

    double targetDistance = 0.0;

    double startingLeftDistance;
    double startingRightDistance;

    Supplier<Double> xInput;
    Supplier<Double> yInput;
    Supplier<Double> rotInput;
    Supplier<Double> multiplierInput;
    boolean FieldRelative;
    Supplier<Boolean> runAutoAlign;

    private double maxVelocityMultiplier = 1;

    boolean autoAlignActive = false;

    double DeadBand = 0.08;
    double RotDeadBand = 0.01;

    public PIDController rotPid;
    public PIDController latPid;

    OptionalInt currentTrackedTarget = OptionalInt.of(0);

    public DriveRobotWithNoteAlign(Drive driveSub, Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot,
            Supplier<Double> multiplier, boolean FieldRelative, Vision visionSub, Supplier<Boolean> runAutoAlign) {

        this.driveSub = driveSub;
        this.xInput = x;
        this.yInput = y;
        this.rotInput = rot;
        this.multiplierInput = multiplier;
        this.FieldRelative = FieldRelative;
        this.visionSub = visionSub;
        this.runAutoAlign = runAutoAlign;

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
    }

    @Override
    public void execute() {

        double speedX;
        double speedY;
        double speedRot;

        speedX = MathUtil.applyDeadband(yInput.get(), DeadBand) * (DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier);
        speedY = MathUtil.applyDeadband(xInput.get(), DeadBand) * (DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier);
        speedRot = MathUtil.applyDeadband(rotInput.get(), 2 * DeadBand) * (DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier);

        // Converts from old range (1 to -1) to desired range (1 to 0.4, 4.5 m/s to 1.8 m/s)
        maxVelocityMultiplier = (((multiplierInput.get() + 1) * (1 - 0.4)) / 2) + 0.4;

        if (autoAlignActive) {
            if (visionSub.noteIsVisible()) {
                var rotOffset = visionSub.getNoteDistanceFromCenter(); // CHANGE: need to convert this to degrees?

                speedRot = rotPid.calculate(rotOffset);
                Logger.recordOutput("Align/targetYaw", visionSub.getNoteDistanceFromCenter());
                Logger.recordOutput("Align/proposedRot", speedRot);

            } else {
                speedRot = 0.0;
                rotPid.reset();
                latPid.reset();
            }
        } else {
            rotPid.reset();
            latPid.reset();
        }

        driveSub.drive(speedX, speedY, speedRot, FieldRelative);

        /* LOGGING */
        Logger.recordOutput("Align/YawInput", speedRot);
        Logger.recordOutput("Align/XSpeed", speedX);
        Logger.recordOutput("Align/YSpeed", speedY);
        Logger.recordOutput("Align/XInput", xInput.get());
        Logger.recordOutput("Align/yInput", yInput.get());
        Logger.recordOutput("Align/rotInput", rotInput.get());

        Logger.recordOutput("Drive/Multipler", maxVelocityMultiplier);

    }

    @Override
    public void end(boolean interrupted) {
        // driveSub.drive(0, 0, 0, true); // Commented this out cause I don't want this to stop us in auto
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(visionSub.getNoteDistanceFromCenter()) <= Constants.AutoConstants.NoteAlignTolerance) {
            return true;
        }
        return false;
    }
}