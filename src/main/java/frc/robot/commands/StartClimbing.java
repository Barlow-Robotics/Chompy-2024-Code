// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterMount;
import frc.robot.subsystems.ShooterMount.ShooterMountState;

public class StartClimbing extends SetShooterMountPosition {
    /** Creates a new StartClimbing. */
    public StartClimbing(ShooterMount shooterMountSub) {
        super(shooterMountSub, ShooterMountState.Preclimb);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (shooterMountSub.getShooterMountState() == ShooterMountState.Preclimb) {
            this.desiredState = ShooterMountState.Climb;
        } else {
            this.desiredState = ShooterMountState.Preclimb;
        }
        shooterMountSub.setDesiredState(desiredState);
    }

}
