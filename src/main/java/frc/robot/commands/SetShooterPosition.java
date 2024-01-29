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
                shooterPositionSub.setShooterPosState(ShooterPositionState.MovingToPosition);
                desiredAngle = ShooterConstants.SpeakerAngle;
                desiredHeight = ElevatorConstants.SpeakerHeight;
                shooterPositionSub.setShooterPosState(ShooterPositionState.Speaker);
                break;
            case Amp:
                shooterPositionSub.setShooterPosState(ShooterPositionState.MovingToPosition);
                desiredAngle = ShooterConstants.AmpAngle;
                desiredHeight = ElevatorConstants.AmpHeight;
                shooterPositionSub.setShooterPosState(ShooterPositionState.Amp);
                break;
            case IntakeFromSource:
                shooterPositionSub.setShooterPosState(ShooterPositionState.MovingToPosition);
                desiredAngle = ShooterConstants.IntakeFromSourceAngle;
                desiredHeight = ElevatorConstants.IntakeFromSourceHeight;
                shooterPositionSub.setShooterPosState(ShooterPositionState.IntakeFromSource);
                break;
            case IntakeFromFloor:
                shooterPositionSub.setShooterPosState(ShooterPositionState.MovingToPosition);
                desiredAngle = ShooterConstants.IntakeFromFloorAngle;
                desiredHeight = ElevatorConstants.IntakeFromFloorHeight;
                shooterPositionSub.setShooterPosState(ShooterPositionState.IntakeFromFloor);
                break;
            case Trap:
                shooterPositionSub.setShooterPosState(ShooterPositionState.MovingToPosition);
                shooterPositionSub.setAngle(ShooterConstants.TrapAngle);
                shooterPositionSub.setShooterPosState(ShooterPositionState.Trap);
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
