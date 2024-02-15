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
  //  private Vision visionSub;
    private double desiredAngle;
    private double desiredHeight;
   // private TargetToAlign desiredTarget;

  public SetShooterMountPosition(ShooterMount shooterMountSub, ShooterMountState desiredState) {
        this.shooterMountSub = shooterMountSub;
        this.desiredState = desiredState;
    //    this.visionSub = visionSub;

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
       //         desiredTarget = TargetToAlign.Speaker;
                break;
            case Amp:
                desiredAngle = ShooterMountConstants.AmpAngle;
                desiredHeight = ShooterMountConstants.AmpHeight;
         //       desiredTarget = TargetToAlign.Amp;
                break;
            case SourceIntake:
                desiredAngle = ShooterMountConstants.SourceIntakeAngle;
                desiredHeight = ShooterMountConstants.SourceIntakeHeight;
           //     desiredTarget = TargetToAlign.Source;
                break;
            case FloorIntake:
                desiredAngle = ShooterMountConstants.FloorIntakeAngle;
                desiredHeight = ShooterMountConstants.FloorIntakeHeight;
                break;
                // LT and EH added code for climb - see comments in ShooterMount
            case PreClimb:  
                desiredAngle = ShooterMountConstants.TrapAngle;
                // no break b/c wants to go to PreTrap also - Ed
            case PreTrap:    
                desiredHeight = ShooterMountConstants.TrapHeight;
                break;
            case Climb:  // pull-up
                desiredHeight = ShooterMountConstants.MinHeight;
                break;
            case Trap:
                break;
        }
        Logger.recordOutput("ShooterMount/DesiredAngle", desiredAngle);
        Logger.recordOutput("ShooterMount/DesiredHeight", desiredHeight);
    }

    @Override
    public void execute() {
        shooterMountSub.setAngle(desiredAngle);
        shooterMountSub.setHeightInches(desiredHeight);
       // visionSub.alignTo(desiredTarget);
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
        // if(shooterMountSub.isAtBottom() && desiredHeight != 0/*  hall effect is true */) {
        //     shooterMountSub.stop();
        //     shooterMountSub.setBasePosition(0);
        // }
        return false;        
    }
}
