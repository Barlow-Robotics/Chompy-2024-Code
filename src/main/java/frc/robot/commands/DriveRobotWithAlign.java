//Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;

import java.util.OptionalDouble;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class DriveRobotWithAlign extends Command {
    Drive driveSub;
    Vision visionSub;

    Supplier<Double> xInput;
    Supplier<Double> yInput;
    Supplier<Double> rotInput;
    Supplier<Double> multiplierInput;
    boolean FieldRelative;
    Supplier<Boolean> runAutoAlign;

    // private double leftVelocity;
    // private double rightVelocity;
    // private double adjustment;
    // private int missedFrames = 0;

    private double maxVelocityMultiplier = 1;

    // private double error;

    Trigger autoAlignButton;

    double DeadBand = 0.08;
    double RotDeadBand = 0.01;

    public PIDController pid;

    public String selectedTarget = "None";

    public DriveRobotWithAlign(Drive driveSub, Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot,
            Supplier<Double> multiplier, boolean FieldRelative, Vision visionSub, Supplier<Boolean> runAutoAlign) {

        this.driveSub = driveSub;
        this.xInput = x;
        this.yInput = y;
        this.rotInput = rot;
        this.multiplierInput = multiplier;
        this.FieldRelative = FieldRelative;
        this.visionSub = visionSub;
        this.runAutoAlign = runAutoAlign;

        pid = new PIDController(
                DriveConstants.AutoAlignKP,
                DriveConstants.AutoAlignKI,
                DriveConstants.AutoAlignKD);

        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
        pid.reset();
        visionSub.chooseBestTarget();
    }

    // add key for auto align button

    @Override
    public void execute() {
        // Since the coordinate systems differ from the controller (x is left right and
        // y is fwd back) and the chassis (positive X is forward, Positive Y is left), we use the
        // controller X input as the drive Y input and the controller Y input as the drive X input.

        boolean autoAlignEnabled = runAutoAlign.get();

        var alignYawControl = 0.0;

        // Converts from old range (1 to -1) to desired range (1 to 0.5)
        maxVelocityMultiplier = (((multiplierInput.get() + 1) * 0.5) / 2) + 0.5;

        if (autoAlignEnabled) {
            var OffSet = (visionSub.getTargetOffSet());
            if (OffSet.isPresent()) {
                alignYawControl = pid.calculate(-OffSet.getAsDouble());
                Logger.recordOutput("Align/Offset", OffSet.getAsDouble());
            }
        }

        // if (autoAlignEnabled) {
        //     rawRot = alignYawControl;
        // }

        double speedX = MathUtil.applyDeadband(yInput.get(), DeadBand) * (DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier);
        double speedY = MathUtil.applyDeadband(xInput.get(), DeadBand) * (DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier);
        double speedRot = MathUtil.applyDeadband(rotInput.get(), 2 * DeadBand) * DriveConstants.MaxDriveableVelocity;

        driveSub.drive(speedX, speedY, speedRot, FieldRelative);

        /* LOGGING */

        Logger.recordOutput("Align/YawInput", speedRot);
        Logger.recordOutput("Align/XSpeed", speedX);
        Logger.recordOutput("Align/YSpeed", speedY);

        Logger.recordOutput("Drive/Multipler", maxVelocityMultiplier);
    }

    public OptionalDouble getTargetOffSet() {
        if (Vision.allDetectedTargets != null) {
            if (Vision.activeAlignTarget.isPresent()) {
                for (PhotonTrackedTarget target : Vision.allDetectedTargets) {
                    if (target.getFiducialId() == Vision.activeAlignTarget.getAsInt()) {
                        Logger.recordOutput("vision/targetY", target.getBestCameraToTarget().getY());
                        Logger.recordOutput("vision/targetYaw", target.getYaw());
                        return OptionalDouble.of(target.getBestCameraToTarget().getY());
                    }
                }

                // getTargetTranslationOffSet getRotation???
            }
        }
        return OptionalDouble.empty();
        /*
         * if (target != null) {
         * return OptionalDouble.of(target.getBestCameraToTarget().getY());
         * }else{
         * return OptionalDouble.empty();
         */
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