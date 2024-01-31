// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterPositionConstants;
import frc.robot.subsystems.ShooterPosition;
import frc.robot.subsystems.ShooterPosition.ShooterPositionState;

public class SetShooterPosition extends Command {
    
    ShooterPosition shooterPositionSub;
    ShooterPositionState desiredState;
    double desiredAngle;
    double desiredHeight;
    public static double desiredShooterVelocity = 0; 
    public static double desiredIndexVelocity = 0;

  public SetShooterPosition(ShooterPosition shooterAngleSub, ShooterPositionState desiredState) {
        this.shooterPositionSub = shooterAngleSub;
        this.desiredState = desiredState;

        addRequirements(shooterAngleSub);
    }

    @Override
    public void initialize() {
        switch (desiredState) {
            case Speaker:
                shooterPositionSub.setShooterPosState(ShooterPositionState.MovingToPosition);
                desiredAngle = ShooterPositionConstants.SpeakerAngle;
                desiredHeight = ShooterPositionConstants.SpeakerHeight;
                desiredShooterVelocity = ShooterConstants.SpeakerVelocity;
                desiredIndexVelocity = ShooterConstants.IndexRPM;
                shooterPositionSub.setShooterPosState(ShooterPositionState.Speaker);
                break;
            case Amp:
                shooterPositionSub.setShooterPosState(ShooterPositionState.MovingToPosition);
                desiredAngle = ShooterPositionConstants.AmpAngle;
                desiredHeight = ShooterPositionConstants.AmpHeight;
                desiredShooterVelocity = ShooterConstants.AmpVelocity;
                desiredIndexVelocity = ShooterConstants.IndexRPM;
                shooterPositionSub.setShooterPosState(ShooterPositionState.Amp);
                break;
            case SourceIntake:
                shooterPositionSub.setShooterPosState(ShooterPositionState.MovingToPosition);
                desiredAngle = ShooterPositionConstants.SourceIntakeAngle;
                desiredHeight = ShooterPositionConstants.SourceIntakeHeight;
                desiredShooterVelocity = ShooterConstants.SourceIntakeVelocity;
                desiredIndexVelocity = -ShooterConstants.IndexRPM;
                shooterPositionSub.setShooterPosState(ShooterPositionState.SourceIntake);
                break;
            case FloorIntake:
                shooterPositionSub.setShooterPosState(ShooterPositionState.MovingToPosition);
                desiredAngle = ShooterPositionConstants.FloorIntakeAngle;
                desiredHeight = ShooterPositionConstants.FloorIntakeHeight;
                desiredShooterVelocity = ShooterConstants.FloorIntakeVelocity;
                desiredIndexVelocity = -ShooterConstants.IndexRPM;
                shooterPositionSub.setShooterPosState(ShooterPositionState.FloorIntake);
                break;
            case Trap:
                shooterPositionSub.setShooterPosState(ShooterPositionState.MovingToPosition);
                shooterPositionSub.setAngle(ShooterPositionConstants.TrapAngle);
                desiredShooterVelocity = ShooterConstants.TrapVelocity;
                desiredIndexVelocity = ShooterConstants.IndexRPM;
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
        return shooterPositionSub.isWithinPositionTolerance(desiredAngle, desiredHeight);
    }
}
