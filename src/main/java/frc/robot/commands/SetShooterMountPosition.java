// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterMountConstants;
import frc.robot.subsystems.ShooterMount;
import frc.robot.subsystems.ShooterMount.ShooterMountState;
import frc.robot.subsystems.Vision; 
import frc.robot.subsystems.Vision.TargetToAlign;

public class SetShooterMountPosition extends Command {
    
    private ShooterMount shooterMountSub;
    private ShooterMountState desiredState;
    private Vision visionSub;
    private double desiredAngle;
    private double desiredHeight;
    private TargetToAlign desiredTarget;

  public SetShooterMountPosition(ShooterMount shooterMountSub, ShooterMountState desiredState, Vision visionSub) {
        this.shooterMountSub = shooterMountSub;
        this.desiredState = desiredState;
        this.visionSub = visionSub;
        addRequirements(shooterMountSub);
    }

    @Override
    public void initialize() {
        shooterMountSub.setShooterPosState(ShooterMountState.MovingToPosition);
        switch (desiredState) {
            case MovingToPosition:   // LT added to remove a warning.  assuming not doing anything here.
                break; 
            case Speaker:
                desiredAngle = ShooterMountConstants.SpeakerAngle;
                desiredHeight = ShooterMountConstants.SpeakerHeight;
                desiredTarget = TargetToAlign.Speaker;
                break;
            case Amp:
                desiredAngle = ShooterMountConstants.AmpAngle;
                desiredHeight = ShooterMountConstants.AmpHeight;
                desiredTarget = TargetToAlign.Amp;
                break;
            case SourceIntake:
                desiredAngle = ShooterMountConstants.SourceIntakeAngle;
                desiredHeight = ShooterMountConstants.SourceIntakeHeight;
                desiredTarget = TargetToAlign.Source;
                break;
            case FloorIntake:
                desiredAngle = ShooterMountConstants.FloorIntakeAngle;
                desiredHeight = ShooterMountConstants.FloorIntakeHeight;
                break;
            case Climb:
                desiredAngle = ShooterMountConstants.TrapAngle;
<<<<<<< Updated upstream
                desiredHeight = ShooterMountConstants.MaxHeightInches;
                if(shooterMountSub.isWithinPositionTolerance(desiredAngle, desiredHeight)) {
                    desiredHeight = ShooterMountConstants.MinHeight;
                }
=======
                // no break b/c wants to go to PreTrap also - Ed
            case PreTrap:    
                desiredHeight = ShooterMountConstants.TrapHeight;
                break;
            case Climb:  // pull-up
                desiredHeight = ShooterMountConstants.StartingHeight;
                break;
            case Trap:
                break;
>>>>>>> Stashed changes
        }
        Logger.recordOutput("ShooterMount/Angle/DesiredAngle", desiredAngle);
        Logger.recordOutput("ShooterMount/Height/DesiredHeightInches", desiredHeight);
    }

    @Override
    public void execute() {
        shooterMountSub.setAngle(desiredAngle);
        shooterMountSub.setHeightInches(desiredHeight);
<<<<<<< Updated upstream
        shooterMountSub.setShooterPosState(desiredState);
=======
>>>>>>> Stashed changes
        if ( desiredTarget != null ) {
            visionSub.alignTo(desiredTarget);  
        }
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
