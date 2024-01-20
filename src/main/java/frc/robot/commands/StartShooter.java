// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class StartShooter extends Command {

  Shooter shooterSub;

  public StartShooter(Shooter s) {
    shooterSub = s;
    addRequirements(shooterSub);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    shooterSub.startShooter();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
