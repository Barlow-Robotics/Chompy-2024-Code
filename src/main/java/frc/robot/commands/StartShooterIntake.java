// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterMount;
import frc.robot.subsystems.ShooterMount.ShooterMountState;

public class StartShooterIntake extends Command {

    Shooter shooterSub;
    FloorIntake floorIntakeSub;
    ShooterMount shooterMountSub;

    public StartShooterIntake(Shooter shooterSub, FloorIntake floorIntakeSub, ShooterMount shooterMountSub) {
        this.shooterSub = shooterSub;
        this.floorIntakeSub = floorIntakeSub;
        this.shooterMountSub = shooterMountSub;
        addRequirements(shooterSub, floorIntakeSub, shooterMountSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // Check to make sure you don't accidentally try to intake a second note
        if ((shooterMountSub.getShooterPosState() == ShooterMountState.SourceIntake ||
                shooterMountSub.getShooterPosState() == ShooterMountState.FloorIntake)
                && shooterSub.isNoteLoaded()) {
            shooterSub.stop();
            floorIntakeSub.stop();
            return;
        }

        shooterSub.start(SetShooterMountPosition.desiredShooterVelocity, SetShooterMountPosition.desiredIndexVelocity);
        if (shooterMountSub.getShooterPosState() == ShooterMountState.FloorIntake) {
            floorIntakeSub.start();
            if (shooterSub.isNoteLoaded()) {
                shooterSub.stop();
                floorIntakeSub.stop();
            }
        } else if (shooterMountSub.getShooterPosState() == ShooterMountState.SourceIntake) {
            if (shooterSub.isNoteLoaded()) {
                shooterSub.stop();
            }
        } else {
            floorIntakeSub.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSub.stop();
        floorIntakeSub.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
