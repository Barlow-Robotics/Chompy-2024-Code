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
                shooterSub.setSpeed(0);
            case Speaker:
                shooterSub.setSpeed(ShooterConstants.SpeakerVelocity);
                break;
            case Amp:
                shooterSub.setSpeed(ShooterConstants.AmpVelocity);
                break;
            case Source:
                shooterSub.setSpeed(ShooterConstants.SourceIntakeVelocity);
                break;
            case Chassis:
                shooterSub.setSpeed(ShooterConstants.ShooterFloorIntakeVelocity);
                break;
            case Trapdoor:
                shooterSub.setSpeed(ShooterConstants.TrapdoorVelocity);
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
