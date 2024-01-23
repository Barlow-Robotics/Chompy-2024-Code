// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterVelState;
import frc.robot.Constants.ShooterConstants;

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
                shooterSub.setVelocity(0);
            case Speaker:
                shooterSub.setVelocity(ShooterConstants.SpeakerVelocity);
                break;
            case Amp:
                shooterSub.setVelocity(ShooterConstants.AmpVelocity);
                break;
            case IntakeFromSource:
                shooterSub.setVelocity(ShooterConstants.SourceIntakeVelocity);
                break;
            case IntakeFromFloor:
                shooterSub.setVelocity(ShooterConstants.ShooterFloorIntakeVelocity);
                break;
            case Trap:
                shooterSub.setVelocity(ShooterConstants.TrapVelocity);
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
