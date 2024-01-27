// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterPosition;
import frc.robot.subsystems.ShooterPosition.ShooterPositionState;

public class SetShooterPosition extends Command {
    
    ShooterPosition shooterPositionSub;
    ShooterPositionState desiredState;
    double desiredAngle;
    double desiredHeight;


  public SetShooterPosition(ShooterPosition shooterAngleSub, ShooterPositionState shooterPositionState) {
        this.shooterPositionSub = shooterAngleSub;
        this.desiredState = shooterPositionState;

        addRequirements(shooterAngleSub);
    }

    @Override
    public void initialize() {
        switch (desiredState) {
            case Speaker:
                desiredAngle = ShooterConstants.SpeakerAngle;
                desiredHeight = ElevatorConstants.SpeakerHeight;
                break;
            case Amp:
                desiredAngle = ShooterConstants.AmpAngle;
                desiredHeight = ElevatorConstants.AmpHeight;
                break;
            case IntakeFromSource:
                desiredAngle = ShooterConstants.IntakeFromSourceAngle;
                desiredHeight = ElevatorConstants.IntakeFromSourceHeight;
                break;
            case IntakeFromFloor:
                desiredAngle = ShooterConstants.IntakeFromFloorAngle;
                desiredHeight = ElevatorConstants.IntakeFromFloorHeight;
                break;
            case Trap:
                shooterPositionSub.setAngle(ShooterConstants.TrapAngle);
                break;
        }
    }

    @Override
    public void execute() {
        shooterPositionSub.setAngle(desiredAngle);
        shooterPositionSub.setHeight(desiredHeight);
    }

    @Override
    public void end(boolean interrupted) {
        shooterPositionSub.shooterPosState = desiredState;
    }

    @Override
    public boolean isFinished() {
        return (shooterPositionSub.getAngle() == desiredAngle) && (shooterPositionSub.getHeight() == desiredHeight);
    }
}
