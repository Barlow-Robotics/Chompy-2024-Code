// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterPositionConstants;
import frc.robot.subsystems.ShooterPosition;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ShooterPosition.ShooterPositionState;
import frc.robot.subsystems.Vision.TargetToAlign;

public class SetMouthPosition extends Command {
    
    private ShooterPosition shooterPositionSub;
    private Vision visionSub;
    private ShooterPositionState desiredState;
    private double desiredAngle;
    private double desiredHeight;
    public static double desiredShooterVelocity = ShooterConstants.FloorRPM; 
    public static double desiredIndexVelocity = ShooterConstants.IndexRPM;
    private TargetToAlign desiredTarget;

  public SetMouthPosition(ShooterPosition shooterAngleSub, ShooterPositionState desiredState, Vision visionSub) {
        this.shooterPositionSub = shooterAngleSub;
        this.desiredState = desiredState;
        this.visionSub = visionSub;

        addRequirements(shooterAngleSub);
    }

    @Override
    public void initialize() {
        shooterPositionSub.setShooterPosState(ShooterPositionState.MovingToPosition);
        switch (desiredState) {
            case Speaker:
                desiredAngle = ShooterPositionConstants.SpeakerAngle;
                desiredHeight = ShooterPositionConstants.SpeakerHeight;
                desiredShooterVelocity = ShooterConstants.SpeakerRPM;
                desiredIndexVelocity = ShooterConstants.IndexRPM;
                desiredTarget = TargetToAlign.Speaker;
                break;
            case Amp:
                desiredAngle = ShooterPositionConstants.AmpAngle;
                desiredHeight = ShooterPositionConstants.AmpHeight;
                desiredShooterVelocity = ShooterConstants.AmpRPM;
                desiredIndexVelocity = ShooterConstants.IndexRPM;
                desiredTarget = TargetToAlign.Amp;
                break;
            case SourceIntake:
                desiredAngle = ShooterPositionConstants.SourceIntakeAngle;
                desiredHeight = ShooterPositionConstants.SourceIntakeHeight;
                desiredShooterVelocity = ShooterConstants.SourceRPM;
                desiredIndexVelocity = -ShooterConstants.IndexRPM;
                desiredTarget = TargetToAlign.Source;
                break;
            case FloorIntake:
                desiredAngle = ShooterPositionConstants.FloorIntakeAngle;
                desiredHeight = ShooterPositionConstants.FloorIntakeHeight;
                desiredShooterVelocity = ShooterConstants.FloorRPM;
                desiredIndexVelocity = -ShooterConstants.IndexRPM;
                break;
            case Trap:
                desiredShooterVelocity = ShooterConstants.TrapRPM;
                desiredIndexVelocity = ShooterConstants.IndexRPM;
                break;
        }
        Logger.recordOutput("ShooterPosition/DesiredAngle", desiredAngle);
        Logger.recordOutput("ShooterPosition/DesiredAngle", desiredHeight);
    }

    @Override
    public void execute() {
        shooterPositionSub.setAngle(desiredAngle);
        shooterPositionSub.setHeight(desiredHeight);
        visionSub.alignTo(desiredTarget);
    }

    @Override
    public void end(boolean interrupted) {
        shooterPositionSub.shooterPosState = ShooterPositionState.Interrupted;
    }

    @Override
    public boolean isFinished() {
        if(shooterPositionSub.isWithinPositionTolerance(desiredAngle, desiredHeight)) {
            shooterPositionSub.setShooterPosState(desiredState);
            return true;
        }
        return false;        
    }
}
