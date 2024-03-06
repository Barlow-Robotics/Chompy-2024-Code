// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterMount;
import frc.robot.subsystems.ShooterMount.ShooterMountState;

public class SetShooterMountPosition extends Command {

    private ShooterMount shooterMountSub;
    private ShooterMountState desiredState;

    public SetShooterMountPosition(ShooterMount shooterMountSub, ShooterMountState desiredState) {
        this.shooterMountSub = shooterMountSub;
        this.desiredState = desiredState;
        addRequirements(shooterMountSub);
    }

    @Override
    public void initialize() {
        shooterMountSub.setDesiredState(desiredState);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override                                                                                                                                                                
    public boolean isFinished() {
        return shooterMountSub.hasCompletedMovement() ;
    }   

}
