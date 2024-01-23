// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterAngle.ShooterAngleState;

public class SetShooterPosition extends Command {
    
    Elevator elevatorSub;
    ShooterAngle shooterAngleSub;
    ShooterAngleState state;

  public SetShooterPosition(ShooterAngle shooterAngleSub, ShooterAngleState state) {
        this.shooterAngleSub = shooterAngleSub;
        this.state = state;
        addRequirements(shooterAngleSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        switch (state) {
            case Speaker:
                shooterAngleSub.setAngle(0);
                break;
            case Amp:
                shooterAngleSub.setAngle(ShooterConstants.AmpAngle);
                break;
            case Source:
                shooterAngleSub.setAngle(ShooterConstants.SourceIntakeAngle);
                break;
            case Chassis:
                shooterAngleSub.setAngle(ShooterConstants.ChassisIntakeAngle);
                break;
            case Trapdoor:
                shooterAngleSub.setAngle(ShooterConstants.TrapDoorAngle);
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
