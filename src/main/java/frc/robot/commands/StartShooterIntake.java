// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPosition;
import frc.robot.subsystems.ShooterPosition.ShooterPositionState;

public class StartShooterIntake extends Command {

    Shooter shooterSub;
    FloorIntake floorIntakeSub;
    ShooterPosition shooterPositionSub;

    public StartShooterIntake(Shooter shooterSub, FloorIntake floorIntakeSub, ShooterPosition shooterPositionSub) {
        this.shooterSub = shooterSub;
        this.floorIntakeSub = floorIntakeSub;
        this.shooterPositionSub = shooterPositionSub;
        addRequirements(shooterSub, floorIntakeSub, shooterPositionSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // Check to make sure you don't accidentally try to intake a second note
        if ((shooterPositionSub.shooterPosState == ShooterPositionState.SourceIntake ||
                shooterPositionSub.shooterPosState == ShooterPositionState.FloorIntake)
                && shooterSub.isNoteLoaded())
            return;

        shooterSub.setVelocity(SetMouthPosition.desiredShooterVelocity, SetMouthPosition.desiredIndexVelocity);
        if (shooterPositionSub.shooterPosState == ShooterPositionState.FloorIntake) {
            floorIntakeSub.startIntaking();
            if (shooterSub.isNoteLoaded()) {
                shooterSub.stopMotors();
                floorIntakeSub.stopIntaking();
            }
        } else if (shooterPositionSub.shooterPosState == ShooterPositionState.SourceIntake) {
            if (shooterSub.isNoteLoaded()) {
                shooterSub.stopMotors();
            }
        } else {
            floorIntakeSub.stopIntaking();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSub.stopMotors();
        floorIntakeSub.stopIntaking();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
