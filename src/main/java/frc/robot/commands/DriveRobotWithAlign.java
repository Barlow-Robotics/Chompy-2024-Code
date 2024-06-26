//Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;

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
import frc.robot.subsystems.ShooterMount;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ShooterMount.ShooterMountState;

import java.util.Optional;

public class DriveRobotWithAlign extends Command {
    Drive driveSub;
    Vision visionSub;
    ShooterMount shooterMountSub;

    Supplier<Double> xInput;
    Supplier<Double> yInput;
    Supplier<Double> rotInput;
    Supplier<Double> multiplierInput;
    boolean FieldRelative;
    Supplier<Boolean> runAutoAlign;

    private double maxVelocityMultiplier = 1;

    // private double error;

    // Trigger autoAlignButton;

    boolean autoAlignActive = false ;

    double DeadBand = 0.08;
    double RotDeadBand = 0.01;

    public PIDController rotPid;
    public PIDController latPid;

    public String selectedTarget = "None";

    OptionalInt currentTrackedTarget = OptionalInt.of(0) ;

    // AprilTagFieldLayout aprilTagFieldLayout ;

    public DriveRobotWithAlign(Drive driveSub, Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot,
            Supplier<Double> multiplier, boolean FieldRelative, Vision visionSub, ShooterMount shooterMountSub, Supplier<Boolean> runAutoAlign) {

        this.driveSub = driveSub;
        this.xInput = x;
        this.yInput = y;
        this.rotInput = rot;
        this.multiplierInput = multiplier;
        this.FieldRelative = FieldRelative;
        this.visionSub = visionSub;
        this.shooterMountSub = shooterMountSub;
        this.runAutoAlign = runAutoAlign;

        rotPid = new PIDController(
                DriveConstants.AutoAlignRotKP,
                DriveConstants.AutoAlignRotKI,
                DriveConstants.AutoAlignRotKD);


        // AprilTagFieldLayout layout;
        // try {
        //     layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        //     var alliance = DriverStation.getAlliance();
        //     if (alliance.isPresent()) {
        //         layout.setOrigin(
        //                 DriverStation.getAlliance().get() == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
        //                         : OriginPosition.kRedAllianceWallRightSide);
        //     } else {
        //         // default this for now
        //         layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        //     }

        // } catch (IOException e) {
        //     DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
        //     layout = null;
        // }
        // this.aprilTagFieldLayout = layout;


        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
        rotPid.reset();
        latPid.reset();
        // visionSub.chooseBestTarget();
    }

    @Override
    public void execute() {
        // Since the coordinate systems differ from the controller (x is left right and
        // y is fwd back) and the chassis (positive X is forward, Positive Y is left), we use the
        // controller X input as the drive Y input and the controller Y input as the drive X input.

        double speedX;
        double speedY;
        double speedRot;

        speedX = MathUtil.applyDeadband(yInput.get(), DeadBand) * (DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier);
        speedY = MathUtil.applyDeadband(xInput.get(), DeadBand) * (DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier);
        speedRot = MathUtil.applyDeadband(rotInput.get(), 2 * DeadBand) * (DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier);

        // wpk put back after driver trials
        boolean autoAlignEnabled = runAutoAlign.get();
        // boolean autoAlignEnabled = false;

        var alignYawControl = 0.0;
        var alignLatControl = 0.0;

        // Converts from old range (1 to -1) to desired range (1 to 0.4,  4.5 m/s to 1.8 m/s)
        maxVelocityMultiplier = (((multiplierInput.get() + 1) * (1 - 0.4)) / 2) + 0.4;

        if (shooterMountSub.desiredState != ShooterMountState.FloorIntake) { // auto align with april tag 

            // if the button transitions from not pressed to pressed
            if ( autoAlignEnabled && !autoAlignActive ) {
                // get the best target we might be interested in
                rotPid.setSetpoint(6.0);  // wpk need to do better. this works close, but not as well far away
                var bestTarget = visionSub.getBestTrackableTarget() ;
                if ( bestTarget.isPresent()) {
                    currentTrackedTarget = OptionalInt.of(bestTarget.get().getFiducialId())  ;
                } else {
                    currentTrackedTarget = OptionalInt.of(0 ) ;
                }
                Logger.recordOutput("Align/bestTarget", currentTrackedTarget.getAsInt());
            }
            
            autoAlignActive = autoAlignEnabled ;

            if (autoAlignActive) {
                if (currentTrackedTarget.isPresent()) {
                    Optional<PhotonTrackedTarget> target = visionSub.getTarget(currentTrackedTarget.getAsInt());
                    if (target.isPresent()) {
                        var rotOffset = target.get().getYaw();
                        var latOffset = 0.0; // wpk temporary bring back later

                        speedRot = rotPid.calculate(rotOffset);
                        Logger.recordOutput("Align/targetYaw", target.get().getYaw());
                        Logger.recordOutput("Align/proposed rot", speedRot);

                        Pose2d targetPose = visionSub.getLayout().getTagPose(currentTrackedTarget.getAsInt()).get().toPose2d() ;

                        var test = PhotonUtils.getYawToPose(driveSub.getPose(), targetPose) ;
                        Logger.recordOutput("Align/yawToPose", test);
                        // alignLatControl = latPid.calculate(-latOffset);
                        //speedRot = 0.0 ;  // wpk remove after testing
                    } else {
                        speedRot = 0.0;
                        rotPid.reset();
                        latPid.reset();
                    }
                }
            } else {
                rotPid.reset();
                latPid.reset();
            }
        } else { // auto align with notes 
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
        }

        driveSub.drive(speedX, speedY, speedRot, FieldRelative);

        /* LOGGING */
        Logger.recordOutput("Align/YawInput", speedRot);
        Logger.recordOutput("Align/XSpeed", speedX);
        Logger.recordOutput("Align/YSpeed", speedY);
        Logger.recordOutput("Align/YawControl", alignYawControl);
        Logger.recordOutput("Align/LatControl", alignLatControl);
        Logger.recordOutput("Align/XInput", xInput.get());
        Logger.recordOutput("Align/yInput", yInput.get());
        Logger.recordOutput("Align/rotInput", rotInput.get());

        Logger.recordOutput("Drive/Multiplier", maxVelocityMultiplier);


    }


    // public OptionalDouble getTargetOffSet() {
    //     if (visionSub.allDetectedTargets != null) {
    //         if (visionSub.isAligningWithNote()) {
    //             return OptionalDouble.of(visionSub.getNoteDistanceFromCenter());
    //         }
    //         if (visionSub.activeAlignTarget.isPresent()) {
    //             for (PhotonTrackedTarget target : visionSub.allDetectedTargets) {
    //                 if (target.getFiducialId() == visionSub.activeAlignTarget.getAsInt()) {
    //                     Logger.recordOutput("vision/targetY", target.getBestCameraToTarget().getY());
    //                     Logger.recordOutput("vision/targetYaw", target.getYaw());
    //                     return OptionalDouble.of(target.getBestCameraToTarget().getY());
    //                 }
    //             }

    //             // getTargetTranslationOffSet getRotation???
    //         }
    //     }
    //     return OptionalDouble.empty();
    //     /*
    //      * if (target != null) {
    //      * return OptionalDouble.of(target.getBestCameraToTarget().getY());
    //      * }else{
    //      * return OptionalDouble.empty();
    //      */
    // }

    @Override
    public void end(boolean interrupted) {
        driveSub.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}