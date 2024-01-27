// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterVelState;

public class SetShooterVelocity extends Command {

    Shooter shooterSub;
    ShooterVelState state;

    public SetShooterVelocity(Shooter shooterSub, ShooterVelState state) {
        this.shooterSub = shooterSub;
        this.state = state;
        addRequirements(shooterSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        switch (state) {
            case Stopped:
                shooterSub.stopShooting();
                shooterSub.shooterVelState = ShooterVelState.Stopped;
                break;
            case Speaker:
                shooterSub.setVelocity(ShooterConstants.SpeakerVelocity);
                shooterSub.shooterVelState = ShooterVelState.Speaker;
                break;
            case Amp:
                shooterSub.setVelocity(ShooterConstants.AmpVelocity);
                shooterSub.shooterVelState = ShooterVelState.Amp;
                break;
            case IntakeFromSource:
                shooterSub.setVelocity(ShooterConstants.SourceIntakeVelocity);
                shooterSub.shooterVelState = ShooterVelState.IntakeFromSource;
                break;
            case IntakeFromFloor:
                shooterSub.setVelocity(ShooterConstants.ShooterFloorIntakeVelocity);
                shooterSub.shooterVelState = ShooterVelState.IntakeFromFloor;
                break;
            case Trap:
                shooterSub.setVelocity(ShooterConstants.TrapVelocity);
                shooterSub.shooterVelState = ShooterVelState.Trap;
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
