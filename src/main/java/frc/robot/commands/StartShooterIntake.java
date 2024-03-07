// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterMountConstants;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterMount;
import frc.robot.subsystems.ShooterMount.ShooterMountState;

public class StartShooterIntake extends Command {

    Shooter shooterSub;
    FloorIntake floorIntakeSub;
    ShooterMount shooterMountSub;
    double desiredLeftFlywheelMotorRPM = ShooterConstants.LeftIntakeRPM;
    double desiredRightFlywheelMotorRPM = ShooterConstants.RightIntakeRPM;
    double desiredIndexRPM = ShooterConstants.IndexRPM;
    double FloorIntakeAngleWithTolerance = ShooterMountConstants.FloorIntakeAngle + 3;

    public StartShooterIntake(Shooter shooterSub, FloorIntake floorIntakeSub, ShooterMount shooterMountSub) {
        this.shooterSub = shooterSub;
        this.floorIntakeSub = floorIntakeSub;
        this.shooterMountSub = shooterMountSub;
        addRequirements(shooterSub, floorIntakeSub);
    }

    @Override
    public void initialize() {
        if (shooterMountSub.getShooterMountState() == ShooterMountState.SourceIntake ||
                shooterMountSub.getShooterMountState() == ShooterMountState.FloorIntake || 
                shooterMountSub.getAngleCANCoderDegrees() < (FloorIntakeAngleWithTolerance)) { // we're intaking
            desiredIndexRPM = -ShooterConstants.IndexRPM;
            desiredLeftFlywheelMotorRPM = ShooterConstants.LeftIntakeRPM;
            desiredRightFlywheelMotorRPM = ShooterConstants.RightIntakeRPM;
        } else { // we're shooting 
            desiredIndexRPM = ShooterConstants.IndexRPM;
            if (shooterMountSub.getShooterMountState() == ShooterMountState.Amp) {
                desiredLeftFlywheelMotorRPM = ShooterConstants.LeftAmpRPM;
                desiredRightFlywheelMotorRPM = ShooterConstants.RightAmpRPM;
            } else if (shooterMountSub.getShooterMountState() == ShooterMountState.Speaker) {
                desiredLeftFlywheelMotorRPM = ShooterConstants.LeftSpeakerRPM;
                desiredRightFlywheelMotorRPM = ShooterConstants.RightSpeakerRPM;
            } else if (shooterMountSub.getShooterMountState() == ShooterMountState.Ferry) {
                desiredLeftFlywheelMotorRPM = ShooterConstants.LeftFerryRPM;
                desiredRightFlywheelMotorRPM = ShooterConstants.RightFerryRPM;
            }
        }
    }

    @Override
    public void execute() {
        if (shooterMountSub.hasCompletedMovement()) {
            shooterSub.startFlywheels(desiredLeftFlywheelMotorRPM, desiredRightFlywheelMotorRPM);
            if (shooterSub.isWithinFlywheelVelocityTolerance(desiredLeftFlywheelMotorRPM,
                    desiredRightFlywheelMotorRPM)) {
                shooterSub.startIndex(desiredIndexRPM);
            }

            if (shooterMountSub.getShooterMountState() == ShooterMountState.FloorIntake ||
                    shooterMountSub.getAngleCANCoderDegrees() < (FloorIntakeAngleWithTolerance)) {
                floorIntakeSub.start();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooterSub.stop();
            floorIntakeSub.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
