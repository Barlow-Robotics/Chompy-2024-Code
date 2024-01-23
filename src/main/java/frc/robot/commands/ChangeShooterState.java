// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.Constants.ShooterConstants;

public class ChangeShooterState extends Command {

    Shooter shooterSub;
    ShooterState state;

    public ChangeShooterState(Shooter shooterSub, ShooterState state) {
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
                shooterSub.setAngle(ShooterConstants.SpeakerAngle);
                break;
            case Amp:
                shooterSub.setSpeed(ShooterConstants.AmpVelocity);
                shooterSub.setAngle(ShooterConstants.AmpAngle);
                break;
            case Source:
                shooterSub.setSpeed(ShooterConstants.SourceIntakeVelocity);
                shooterSub.setAngle(ShooterConstants.SourceIntakeAngle);
                break;
            case Chassis:
                shooterSub.setSpeed(ShooterConstants.ShooterFloorIntakeVelocity);
                shooterSub.setAngle(ShooterConstants.ShooterFloorIntakeAngle);
                break;
            case Trapdoor:
                // method goes here
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
