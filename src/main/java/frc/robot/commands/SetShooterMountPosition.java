// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

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
            case Speaker:
                desiredAngle = getSpeakerShooterAngle();
                if (desiredAngle == VisionConstants.InvalidAngle) // couldn't find speaker AprilTag
                    desiredAngle = ShooterMountConstants.SpeakerAngle;
                desiredHeight = ShooterMountConstants.SpeakerHeight;
                desiredTarget = TargetToAlign.Speaker;
                break;
            /*
             * old code - before setting a distance-based angle to speaker
             * case Speaker:
             * desiredAngle = ShooterMountConstants.SpeakerAngle;
             * desiredHeight = ShooterMountConstants.SpeakerHeight;
             * desiredTarget = TargetToAlign.Speaker;
             * break;
             */
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
                desiredAngle = ShooterMountConstants.TrapAngle;
                desiredHeight = ShooterMountConstants.MaxHeightInches;
                if (shooterMountSub.isWithinPositionTolerance(desiredAngle, desiredHeight)) {
                    desiredHeight = ShooterMountConstants.StartingHeight;
                }
        }
        Logger.recordOutput("ShooterMount/Angle/DesiredAngle", desiredAngle);
        Logger.recordOutput("ShooterMount/Height/DesiredHeightInches", desiredHeight);
    }

    @Override
    public void execute() {
        shooterMountSub.setAngle(desiredAngle);
        shooterMountSub.setHeightInches(desiredHeight);
        if (desiredTarget != null) {
            visionSub.alignTo(desiredTarget);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (shooterMountSub.isWithinPositionTolerance(desiredAngle, desiredHeight)) {
            shooterMountSub.setShooterPosState(desiredState); // LMT CHANGE? See comment below
            return true;
        }
        return false; // LMT - CHANGE this to true, or based on another condition?
        // As you drive toward the speaker, you want to keep calculating and setting new
        // angles until they shoot
        // or maybe until they let go of the (speaker or action) button. Desired
        // behavior TBD
    }

    public double getSpeakerShooterAngle() {

        double apriltagPitch = visionSub.getSpeakerAprilTagPitch();
        if (apriltagPitch == VisionConstants.InvalidAngle)
            return VisionConstants.InvalidAngle;

        // Angle to speaker = Arctan((SpkrHt - (ElevHtUnext)) / ((ATHt-CamHt) /
        // tan(ATpitch)) )

        // LMT - CHANGE? This should be properly considering 0 angle, but double check.
        // Also - change to atan2 or add math to check for div by 0 error
        return (Math.atan((ShooterMountConstants.MidSpeakerHeight - ShooterMountConstants.ElevatorHeightUnextended)
                / ((ShooterMountConstants.SpeakerAprilTagHeight - ShooterMountConstants.CameraMountHeight) /
                        Math.tan(apriltagPitch))));
    }

}
