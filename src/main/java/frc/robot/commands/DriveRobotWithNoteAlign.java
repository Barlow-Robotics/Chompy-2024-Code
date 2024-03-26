//Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;

import java.util.Optional;
import java.util.OptionalInt;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Vision;

public class DriveRobotWithNoteAlign extends Command {

    final Drive driveSub;
    final Vision visionSub;
    final FloorIntake intake;


    // double targetDistance = 0.0;

    // double startingLeftDistance;
    // double startingRightDistance;

    final Supplier<Double> xInput;
    final Supplier<Double> yInput;
    final Supplier<Double> rotInput;
    final Supplier<Double> multiplierInput;
    final Supplier<Boolean> runAutoAlign;

    // private double maxVelocityMultiplier = 1;
    // boolean FieldRelative;

    final double DeadBand = 0.08;
    final double RotDeadBand = 0.01;
    boolean autoAlignActive = false ;


    private final PIDController tagPID;
    private final PIDController notePID;

    OptionalInt currentTrackedTarget = OptionalInt.of(0);

    public DriveRobotWithNoteAlign(Drive driveSub, Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot,
            Supplier<Double> multiplier, Vision visionSub, FloorIntake intake, Supplier<Boolean> runAutoAlign) {

        this.driveSub = driveSub;
        this.intake = intake ;
        this.xInput = x;
        this.yInput = y;
        this.rotInput = rot;
        this.multiplierInput = multiplier;
        // this.FieldRelative = FieldRelative;
        this.visionSub = visionSub;
        this.runAutoAlign = runAutoAlign;

        tagPID = new PIDController(
                DriveConstants.AutoAlignRotKP,
                DriveConstants.AutoAlignRotKI,
                DriveConstants.AutoAlignRotKD);
        tagPID.setSetpoint(6.0);  


        // wpk may need to tweak the values in constants
        notePID = new PIDController(
                DriveConstants.AutoAlignNoteKP,
                DriveConstants.AutoAlignNoteKI,
                DriveConstants.AutoAlignNoteKD);
        notePID.setSetpoint(0.0);

        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
        tagPID.reset();
        notePID.reset();
    }

    @Override
    public void execute() {

        double speedX;
        double speedY;
        double speedRot;

        // Converts from old range (1 to -1) to desired range (1 to 0.4, 4.5 m/s to 1.8 m/s)
        double maxVelocityMultiplier = (((multiplierInput.get() + 1) * (1 - 0.4)) / 2) + 0.4;

        speedX = MathUtil.applyDeadband(yInput.get(), DeadBand) * (DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier);
        speedY = MathUtil.applyDeadband(xInput.get(), DeadBand) * (DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier);
        speedRot = MathUtil.applyDeadband(rotInput.get(), 2 * DeadBand) * (DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier);

        boolean fieldRelative = true ;

        var autoAlignEnabled = runAutoAlign.get() ;

        // if the button transitions from not pressed to pressed
        if ( autoAlignEnabled && !autoAlignActive ) {
            // get the best target we might be interested in
            var bestTarget = visionSub.getBestTrackableTarget() ;
            if ( bestTarget.isPresent()) {
                currentTrackedTarget = OptionalInt.of(bestTarget.get().getFiducialId())  ;
            } else {
                currentTrackedTarget = OptionalInt.of(0 ) ;
            }
            Logger.recordOutput("Align/bestTarget", currentTrackedTarget.getAsInt());
        }        
        autoAlignActive = autoAlignEnabled ;

        if (autoAlignEnabled) {
            if (intake.isIntaking()) { // chase after the note
                if (visionSub.noteIsVisible()) {

                    // compute magnitude of the speed vector
                    speedX = Math.sqrt(speedX * speedX + speedY + speedY);
                    // negate value to match robot coordinate system
                    var noteOffset = -visionSub.getNoteDistanceFromCenter();
                    speedY = notePID.calculate(noteOffset);
                    fieldRelative = false;
                } else {
                    notePID.reset();
                }
            } else {
                if (currentTrackedTarget.isPresent()) {
                    Optional<PhotonTrackedTarget> target = visionSub.getTarget(currentTrackedTarget.getAsInt());
                    if (target.isPresent()) {
                        var rotOffset = target.get().getYaw();

                        speedRot = tagPID.calculate(rotOffset);
                        Logger.recordOutput("Align/targetYaw", target.get().getYaw());
                        Logger.recordOutput("Align/proposed rot", speedRot);

                        Pose2d targetPose = visionSub.getLayout().getTagPose(currentTrackedTarget.getAsInt()).get()
                                .toPose2d();

                        var test = PhotonUtils.getYawToPose(driveSub.getPose(), targetPose);
                        Logger.recordOutput("Align/yawToPose", test);
                        // alignLatControl = latPid.calculate(-latOffset);
                        // speedRot = 0.0 ; // wpk remove after testing
                    } else {
                        speedRot = 0.0;
                        tagPID.reset();
                    }
                }

            }
            tagPID.reset();
            notePID.reset();
        }

        driveSub.drive(speedX, speedY, speedRot, fieldRelative);

        /* LOGGING */
        Logger.recordOutput("Align/YawInput", speedRot);
        Logger.recordOutput("Align/XSpeed", speedX);
        Logger.recordOutput("Align/YSpeed", speedY);
        Logger.recordOutput("Align/XInput", xInput.get());
        Logger.recordOutput("Align/yInput", yInput.get());
        Logger.recordOutput("Align/rotInput", rotInput.get());
        Logger.recordOutput("Align/NoteVisible", visionSub.noteIsVisible());
        Logger.recordOutput("Align/NoteDistanceFromCenter", visionSub.getNoteDistanceFromCenter());
        Logger.recordOutput("Drive/Multipler", maxVelocityMultiplier);

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}