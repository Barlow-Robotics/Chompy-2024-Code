// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterMountConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ShooterMount;
import frc.robot.subsystems.ShooterMount.ShooterMountState;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.TargetToAlign;

public class SetShooterMountPosition extends Command {

    private ShooterMount shooterMountSub;
    private ShooterMountState desiredState;
    private Vision visionSub;
    private double desiredAngle;
    private double desiredHeight;
    private TargetToAlign desiredTarget;
    private String climbState = "PreClimb";

    public SetShooterMountPosition(ShooterMount shooterMountSub, ShooterMountState desiredState, Vision visionSub) {
        this.shooterMountSub = shooterMountSub;
        this.desiredState = desiredState;
        this.visionSub = visionSub;
        addRequirements(shooterMountSub);
    }

    @Override
    public void initialize() {
        shooterMountSub.setShooterPosState(ShooterMountState.MovingToPosition);
        switch (desiredState) {
            case MovingToPosition: // LT added to remove a warning. assuming not doing anything here.
                break;
            // case Speaker:
            // desiredAngle = getSpeakerShooterAngle();
            // if (desiredAngle == VisionConstants.InvalidAngle) // couldn't find speaker AprilTag
            //     desiredAngle = ShooterMountConstants.SpeakerAngle;
            // desiredHeight = ShooterMountConstants.SpeakerHeight;
            // desiredTarget = TargetToAlign.Speaker;

            // break;

            // old code - before setting a distance-based angle to speaker
            case Speaker:
                desiredAngle = ShooterMountConstants.SpeakerAngle ;
//                desiredAngle = shooterMountSub.getSpeakerShooterAngle() ;
                desiredHeight = ShooterMountConstants.SpeakerHeight;
                desiredTarget = TargetToAlign.Speaker;
                break;

            case Amp:
                desiredAngle = ShooterMountConstants.AmpAngle;
                desiredHeight = ShooterMountConstants.AmpHeight;
                desiredTarget = TargetToAlign.Amp;
                break;
            case SourceIntake:
                desiredAngle = ShooterMountConstants.SourceIntakeAngle;
                desiredHeight = ShooterMountConstants.SourceIntakeHeight;
                desiredTarget = TargetToAlign.Source;
                break;
            case FloorIntake:
                desiredAngle = ShooterMountConstants.FloorIntakeAngle;
                desiredHeight = ShooterMountConstants.FloorIntakeHeight;
                break;
            case Climb:
                if (climbState.equals("PreClimb")) {
                    desiredHeight = ShooterMountConstants.ClimbHeight;
                    if (shooterMountSub.isWithinPositionTolerance(desiredAngle, desiredHeight)) {
                        climbState = "Climb";
                        break;
                    }
                } else if (climbState.equals("Climb")) {
                    desiredHeight = ShooterMountConstants.StartingHeight;
                    if (shooterMountSub.isWithinPositionTolerance(desiredAngle, desiredHeight)) {
                        climbState = "Trap";
                        break;
                    }
                } else if (climbState.equals("Trap")) {
                    desiredAngle = ShooterMountConstants.TrapAngle;
                    desiredHeight = ShooterMountConstants.TrapHeight;
                    if (shooterMountSub.isWithinPositionTolerance(desiredAngle, desiredHeight)) {
                        break;
                    }
                }
                break;
            case ClimbAbort:
                shooterMountSub.stopElevatorMotor();
                shooterMountSub.stopAngleMotor();
                break;
            case Ferry:
                desiredAngle = ShooterMountConstants.FerryAngle;
                desiredHeight = ShooterMountConstants.FerryHeight;
                break;
        }
        Logger.recordOutput("ShooterMount/Angle/DesiredAngle", desiredAngle);
        Logger.recordOutput("ShooterMount/Height/DesiredHeightInches", desiredHeight);
    }

    @Override
    public void execute() {
        if (shooterMountSub.getShooterMountStateAsString() != "ClimbAbort") {
            shooterMountSub.setAngle(desiredAngle);
            shooterMountSub.setHeightInches(desiredHeight);
            // if (desiredTarget != null) {
            //     visionSub.alignTo(desiredTarget);
            // }
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override                                                                                                                                                                
    public boolean isFinished() {
        
        if(shooterMountSub.getShooterMountState() != ShooterMountState.ClimbAbort) {

            if (shooterMountSub.isWithinPositionTolerance(desiredAngle, desiredHeight)) {
                shooterMountSub.setShooterPosState(desiredState); // LMT CHANGE? See comment below
                return true;
            }else{
                return false;
            }
        } else {
            return true;
        } 
    }
    

    
    // public OptionalDouble getSpeakerShooterAngle() {

    //     double result = 0.0;

    //     var target = visionSub.getSpeakerTarget();
    //     if ( target.isEmpty()) {
    //         return OptionalDouble.empty();
    //     }

    //     // deltaY is the difference in height of target (a little over the bottom of speaker opening) and the current
    //     // elevator position.
    //     double height = ShooterMountConstants.MidSpeakerHeight - shooterMountSub.getHeightInches() ;

    //     //deltaX is the horizontal distance to the april tag (and the speaker)
    //     double x = target.get().getBestCameraToTarget().getX() ;
    //     double y = target.get().getBestCameraToTarget().getY() ;
    //     double distance = Math.sqrt(x*x + y*y) ;
        
    //     // Compute the angle and return it.
    //     result = Math.atan2(height, distance); // Make sure X,Y units match
    //     return OptionalDouble.of(result);



    //     // double apriltagPitch = visionSub.getSpeakerAprilTagPitch();
    //     // if (apriltagPitch == VisionConstants.InvalidAngle) {
    //     //     return returnValue;
    //     // }

    //     // else if (apriltagPitch == 0) { // Camera at height of Speaker's AprilTag
    //     //     double horizDistToTarget = visionSub.getSpeakerTargetDistance(); // What if this returns
    //     //                                                                      // VisionConstants.NoTargetDistance???
    //     //     if (horizDistToTarget != VisionConstants.NoTargetDistance) {
    //     //         result =  (Math.atan2(
    //     //                 ShooterMountConstants.MidSpeakerHeight - ShooterMountConstants.ElevatorHeightUnextended,
    //     //                 horizDistToTarget)); // Make sure X,Y units match
    //     //         return OptionalDouble.of( result ) ;
    //     //     }
    //     //     // Need a return value if, for some reason, getSpeakerTargetDistance()
    //     //     // returns NoTargetDistance
    //     //     // Maybe return 0 to keep the shooter horizontal?
    //     //     // Or the initial speaker angle?
    //     // }
    //     // // Angle to speaker = Arctan((SpkrHt - (ElevHtUnext)) / ((ATHt-CamHt) /
    //     // // tan(ATpitch)) )

    //     // // LMT - CHANGE? This should be properly considering 0 angle, but double check.
    //     // // Also - change to atan2 or add math to check for div by 0 error
    //     // result = (Math.atan2((ShooterMountConstants.MidSpeakerHeight - ShooterMountConstants.ElevatorHeightUnextended),
    //     //         ((ShooterMountConstants.SpeakerAprilTagHeight - ShooterMountConstants.CameraMountHeight) /
    //     //                 Math.tan(apriltagPitch))));
    //     // return OptionalDouble.of( result ) ;

    // }

}
