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

    double desiredFlywheelRPM = ShooterConstants.IntakeRPM;
    double desiredIndexRPM = ShooterConstants.IndexRPM;

    public StartShooterIntake(Shooter shooterSub, FloorIntake floorIntakeSub, ShooterMount shooterMountSub) {
        this.shooterSub = shooterSub;
        this.floorIntakeSub = floorIntakeSub;
        this.shooterMountSub = shooterMountSub;
        addRequirements(shooterSub, floorIntakeSub, shooterMountSub);
    }

    @Override
    public void initialize() {
        if (shooterMountSub.getShooterPosState() == ShooterMountState.SourceIntake ||
                shooterMountSub.getShooterPosState() == ShooterMountState.FloorIntake) {
            desiredIndexRPM = -ShooterConstants.IndexRPM;
            desiredFlywheelRPM = ShooterConstants.IntakeRPM;
        } else {
            desiredIndexRPM = ShooterConstants.IndexRPM;
            if (shooterMountSub.getShooterPosState() == ShooterMountState.Amp) {
                desiredFlywheelRPM = ShooterConstants.AmpRPM;
            } else {
                desiredFlywheelRPM = ShooterConstants.SpeakerRPM;
            }
        }
    }

    @Override
    public void execute() {

        // First time through, this checks to make sure you don't accidentally try to intake a second note
        // In subsequent passes, this checks to see if you got the note you intended to intake

        if ((shooterMountSub.getShooterPosState() == ShooterMountState.SourceIntake ||
                shooterMountSub.getShooterPosState() == ShooterMountState.FloorIntake)
                && shooterSub.isNoteLoaded()) {
            shooterSub.stop();
            floorIntakeSub.stop();
            return;
        }

        shooterSub.startFlywheels(desiredFlywheelRPM);
        if (shooterMountSub.getShooterPosState() == ShooterMountState.FloorIntake) {
            floorIntakeSub.start();
            // if (shooterSub.isNoteLoaded()) { // did we get the note we wanted
            //     shooterSub.stop();           // don't need these 3 lines b/c the above will catch it
            //     floorIntakeSub.stop();
            // }
        } else if (shooterMountSub.getShooterPosState() == ShooterMountState.SourceIntake) {
            if (shooterSub.isNoteLoaded()) { // did we get the note we wanted
                shooterSub.stop();
            }
        } else {
            floorIntakeSub.stop();  // why is this needed?
        }
        shooterSub.startIndex(desiredIndexRPM);  // does this restart things when it shouldnt?  e.g. after you've decided to stop it above 
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooterSub.stop();
            floorIntakeSub.stop();
        }
    }

    @Override
    public boolean isFinished() {// Need to CHANGE this
        return false;
    }
}
