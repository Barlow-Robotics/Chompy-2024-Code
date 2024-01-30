// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntake;

public class StopIntake extends Command {
  
  FloorIntake intakeSub;
  
  public StopIntake(FloorIntake intakeSub) {
    this.intakeSub = intakeSub;
    addRequirements(intakeSub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSub.stopIntaking();
    Logger.recordOutput("FloorIntake/isIntaking", intakeSub.isIntaking());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
