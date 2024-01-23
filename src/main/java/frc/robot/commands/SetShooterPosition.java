// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterAngle.ShooterPositionState;

public class SetShooterPosition extends Command {
    
    Elevator elevatorSub;
    ShooterAngle shooterAngleSub;
    ShooterPositionState desiredState;
    double desiredAngle;
    double desiredHeight;


  public SetShooterPosition(ShooterAngle shooterAngleSub, Elevator elevatorSub, ShooterPositionState shooterPositionState) {
        this.shooterAngleSub = shooterAngleSub;
        this.elevatorSub = elevatorSub;
        this.desiredState = shooterPositionState;

        addRequirements(shooterAngleSub, elevatorSub);
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
                shooterAngleSub.setAngle(ShooterConstants.TrapAngle);
                break;
        }
    }

    @Override
    public void execute() {
        shooterAngleSub.setAngle(desiredAngle);
        elevatorSub.setHeight(desiredHeight);
    }

    @Override
    public void end(boolean interrupted) {
        shooterAngleSub.shooterAngleState = desiredState;
    }

    @Override
    public boolean isFinished() {
        return (shooterAngleSub.getAngle() == desiredAngle) && (elevatorSub.getHeight() == desiredHeight);
    }
}
