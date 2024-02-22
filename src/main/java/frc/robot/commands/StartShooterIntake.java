// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterMount;
import frc.robot.subsystems.ShooterMount.ShooterMountState;

public class StartShooterIntake extends Command {

    Shooter shooterSub;
    FloorIntake floorIntakeSub;
    ShooterMount shooterMountSub;
    double desiredLeftFlywheelMotorRPM = ShooterConstants.LeftIntakeRPM;
    double desiredRightFlywheelMotorRPM = ShooterConstants.RightIntakeRPM;
    double desiredIndexRPM = ShooterConstants.IndexRPM;

    public StartShooterIntake(Shooter shooterSub, FloorIntake floorIntakeSub, ShooterMount shooterMountSub) {
        this.shooterSub = shooterSub;
        this.floorIntakeSub = floorIntakeSub;
        this.shooterMountSub = shooterMountSub;
        addRequirements(shooterSub, floorIntakeSub);
    }

    @Override
    public void initialize() {
        if (shooterMountSub.getShooterMountState() == ShooterMountState.SourceIntake ||
                shooterMountSub.getShooterMountState() == ShooterMountState.FloorIntake) {
            desiredIndexRPM = -ShooterConstants.IndexRPM;
            desiredLeftFlywheelMotorRPM = ShooterConstants.LeftIntakeRPM;
            desiredRightFlywheelMotorRPM = ShooterConstants.RightIntakeRPM;
        } else {
            desiredIndexRPM = ShooterConstants.IndexRPM;
            if (shooterMountSub.getShooterMountState() == ShooterMountState.Amp) {
                desiredLeftFlywheelMotorRPM = ShooterConstants.LeftAmpRPM;
                desiredRightFlywheelMotorRPM = ShooterConstants.RightAmpRPM;
            } else {
                desiredLeftFlywheelMotorRPM = ShooterConstants.LeftSpeakerRPM;
                desiredRightFlywheelMotorRPM = ShooterConstants.RightSpeakerRPM;
            }
        }
    }

    @Override
    public void execute() {

        // First time through, this checks to make sure you don't accidentally try to
        // intake a second note
        // In subsequent passes, this checks to see if you got the note you intended to
        // intake

        if ((shooterMountSub.getShooterMountState() == ShooterMountState.SourceIntake ||
                shooterMountSub.getShooterMountState() == ShooterMountState.FloorIntake)
                && shooterSub.isNoteLoaded()) {
            shooterSub.stop();
            floorIntakeSub.stop();
            return;
        }

        shooterSub.startFlywheels(desiredLeftFlywheelMotorRPM, desiredRightFlywheelMotorRPM);
        if (shooterSub.isWithinFlywheelVelocityTolerance(desiredLeftFlywheelMotorRPM, desiredRightFlywheelMotorRPM)) {
            shooterSub.startIndex(desiredIndexRPM);
        }

        if (shooterMountSub.getShooterMountState() == ShooterMountState.FloorIntake) {
            floorIntakeSub.start();
            // if (shooterSub.isNoteLoaded()) { // did we get the note we wanted
            // shooterSub.stop(); // don't need these 3 lines b/c the above will catch it
            // floorIntakeSub.stop();
            // }
        } else if (shooterMountSub.getShooterMountState() == ShooterMountState.SourceIntake) {
            if (shooterSub.isNoteLoaded()) { // did we get the note we wanted
                shooterSub.stop();
            }
        } else {
            floorIntakeSub.stop(); // why is this needed?
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooterSub.stop();
            floorIntakeSub.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
