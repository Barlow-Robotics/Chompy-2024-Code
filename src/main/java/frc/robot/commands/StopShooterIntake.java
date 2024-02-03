// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Shooter;

public class StopShooterIntake extends Command {

   Shooter shooterSub;
    FloorIntake floorIntakeSub;

    public StopShooterIntake(Shooter shooterSub, FloorIntake floorIntakeSub) {
        this.shooterSub = shooterSub;
        this.floorIntakeSub = floorIntakeSub;
        addRequirements(shooterSub, floorIntakeSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterSub.stopMotors();
        floorIntakeSub.stopIntaking();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
