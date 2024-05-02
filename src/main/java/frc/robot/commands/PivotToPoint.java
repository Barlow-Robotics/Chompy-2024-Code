// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class PivotToPoint extends Command {
    Drive driveSub;
    Pose2d targetPose;
    Pose2d robotPose;
    double desiredHeading;
    ProfiledPIDController headingPID;
    double PivotHeadingkP = 0.1;
    double PivotHeadingkI = 0;
    double PivotHeadingkD = 0;

    public PivotToPoint(Pose2d targetPose) {
        this.targetPose = targetPose;
        headingPID = new ProfiledPIDController(PivotHeadingkP, PivotHeadingkI, PivotHeadingkD, new TrapezoidProfile.Constraints(
                DriveConstants.ModuleMaxAngularVelocity, DriveConstants.ModuleMaxAngularAcceleration));
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(driveSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        robotPose = driveSub.getPose();
        double robotAngle = robotPose.getRotation().getRadians();
        double deltaX = robotPose.getX() - targetPose.getX();
        double deltaY = robotPose.getY() - targetPose.getY();

        desiredHeading = Math.atan2(deltaY, deltaX);
        double deltaHeading = desiredHeading - robotAngle;

        // if (deltaHeading > Math.PI) {
        //     deltaHeading -= 2 * Math.PI;
        // } else if (deltaHeading < -Math.PI) {
        //     deltaHeading += 2 * Math.PI;
        // }

        headingPID.setGoal(desiredHeading);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentHeading = driveSub.getPose().getRotation().getDegrees();
        double headingDeterminedAngle = headingPID.calculate(currentHeading);
        driveSub.drive(0, 0, headingDeterminedAngle, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
