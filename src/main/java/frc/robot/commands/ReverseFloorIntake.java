// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntake;

public class ReverseFloorIntake extends Command {

  FloorIntake floorIntakeSub;

  public ReverseFloorIntake(FloorIntake floorIntakeSub) {
    this.floorIntakeSub = floorIntakeSub;
    addRequirements(floorIntakeSub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    floorIntakeSub.reverse();
  }

  @Override
  public void end(boolean interrupted) {
    floorIntakeSub.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
