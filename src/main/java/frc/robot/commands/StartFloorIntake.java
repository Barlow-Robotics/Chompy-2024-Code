// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.FloorIntake;


public class StartFloorIntake extends Command {
  // FloorIntake floorIntakeSub;
  /** Creates a new StartIntake. */
  public StartFloorIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    // floorIntakeSub = f;
    // addRequirements(floorIntakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // floorIntakeSub.startIntaking();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
