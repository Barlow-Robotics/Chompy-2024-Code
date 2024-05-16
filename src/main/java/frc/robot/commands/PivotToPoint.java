// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

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
    // double deltaHeading;
    ProfiledPIDController headingPID;
    double PivotHeadingkP = 3;
    double PivotHeadingkI = 0.0;
    double PivotHeadingkD = 0.0;

    public PivotToPoint(Pose2d targetPose, Drive driveSub) {
        this.targetPose = targetPose;
        this.driveSub = driveSub;
        headingPID = new ProfiledPIDController(PivotHeadingkP, PivotHeadingkI, PivotHeadingkD, new TrapezoidProfile.Constraints(
                DriveConstants.ModuleMaxAngularVelocity, DriveConstants.ModuleMaxAngularAcceleration));
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(driveSub);
        // headingPID.setGoal(0.0);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        robotPose = driveSub.getPose();
        double deltaX = robotPose.getX() - targetPose.getX();
        double deltaY = robotPose.getY() - targetPose.getY();

        desiredHeading = Math.atan2(deltaY, deltaX);
        // double deltaHeading = desiredHeading - robotAngle;

        // if (deltaHeading > Math.PI) {
        //     deltaHeading -= 2 * Math.PI;
        // } else if (deltaHeading < -Math.PI) {
        //     deltaHeading += 2 * Math.PI;
        // }
        // System.out.println("Init = True");

        headingPID.setGoal(desiredHeading-Math.PI);
        // System.out.println("PivotToPoint: Desired Heading: " + deltaHeading + " Robot Heading: " + robotAngle + " Delta Heading: " + deltaHeading);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentHeading = driveSub.getPose().getRotation().getRadians();
        double headingDeterminedAngle = headingPID.calculate(currentHeading);
        driveSub.drive(0, 0, headingDeterminedAngle, false);
        Logger.recordOutput("Drive/PivotToPoint/DesiredHeading", desiredHeading);
        // System.out.println("Running = True");
        // System.out.println("Desired Heading:  " + desiredHeading);
        // System.out.println("Current Heading:  " + currentHeading);


        // Logger.recordOutput("Drive/PivotToPoint/DeltaHeading", deltaHeading);
        Logger.recordOutput("Drive/PivotToPoint/CurrentHeading", currentHeading);
        Logger.recordOutput("Drive/PivotToPoint/HeadingDeterminedAngle", headingDeterminedAngle);
        Logger.recordOutput("Drive/PivotToPoint/HeadingPositionError", headingPID.getPositionError());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSub.drive(0, 0, 0, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        return headingPID.atGoal(); 
        // System.out.println("Error:  " + error);
        // return Math.abs(error) < 0.01;
    }
}
