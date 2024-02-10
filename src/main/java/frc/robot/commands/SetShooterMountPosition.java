// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterMountConstants;
import frc.robot.subsystems.ShooterMount;
import frc.robot.subsystems.ShooterMount.ShooterMountState;

public class SetShooterMountPosition extends Command {
    
    private ShooterMount shooterMountSub;
    private ShooterMountState desiredState;
    private double desiredAngle;
    private double desiredHeight;
    public static double desiredFlywheelVelocity = ShooterConstants.FloorRPM; 
    public static double desiredIndexVelocity = ShooterConstants.IndexRPM;

  public SetShooterMountPosition(ShooterMount shooterAngleSub, ShooterMountState desiredState) {
        this.shooterMountSub = shooterAngleSub;
        this.desiredState = desiredState;

        addRequirements(shooterAngleSub);
    }

    @Override
    public void initialize() {
        shooterMountSub.setShooterPosState(ShooterMountState.MovingToPosition);
        switch (desiredState) {
            case Speaker:
                desiredAngle = ShooterMountConstants.SpeakerAngle;
                desiredHeight = ShooterMountConstants.SpeakerHeight;
                desiredFlywheelVelocity = ShooterConstants.SpeakerRPM;
                desiredIndexVelocity = ShooterConstants.IndexRPM;
                break;
            case Amp:
                desiredAngle = ShooterMountConstants.AmpAngle;
                desiredHeight = ShooterMountConstants.AmpHeight;
                desiredFlywheelVelocity = ShooterConstants.AmpRPM;
                desiredIndexVelocity = ShooterConstants.IndexRPM;
                break;
            case SourceIntake:
                desiredAngle = ShooterMountConstants.SourceIntakeAngle;
                desiredHeight = ShooterMountConstants.SourceIntakeHeight;
                desiredFlywheelVelocity = ShooterConstants.SourceRPM;
                desiredIndexVelocity = -ShooterConstants.IndexRPM;
                break;
            case FloorIntake:
                desiredAngle = ShooterMountConstants.FloorIntakeAngle;
                desiredHeight = ShooterMountConstants.FloorIntakeHeight;
                desiredFlywheelVelocity = ShooterConstants.FloorRPM;
                desiredIndexVelocity = -ShooterConstants.IndexRPM;
                break;
                // LT and EH added code for climb - see comments in ShooterMount
            case PreClimb:  
                desiredAngle = ShooterMountConstants.TrapAngle;
                desiredFlywheelVelocity = 0;
                desiredIndexVelocity = 0;
                // no break b/c wants to go to PreTrap also - Ed
            case PreTrap:    
                desiredHeight = ShooterMountConstants.TrapHeight;
                break;
            case Climb:  // pull-up
                desiredHeight = ShooterMountConstants.MinHeight;
                break;
            case Trap:
                desiredFlywheelVelocity = ShooterConstants.TrapRPM;
                desiredIndexVelocity = ShooterConstants.IndexRPM;
                break;
        }
        Logger.recordOutput("ShooterMount/DesiredAngle", desiredAngle);
        Logger.recordOutput("ShooterMount/DesiredHeight", desiredHeight);
    }

    @Override
    public void execute() {
        shooterMountSub.setAngle(desiredAngle);
        shooterMountSub.setHeightInches(desiredHeight);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if(shooterMountSub.isWithinPositionTolerance(desiredAngle, desiredHeight)) {
            shooterMountSub.setShooterPosState(desiredState);
            return true;
        }
        return false;        
    }
}
