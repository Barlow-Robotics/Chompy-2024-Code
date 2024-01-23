// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class StartShooter extends Command {

  Shooter shooterSub;
  ShooterState state;

  public StartShooter(Shooter s, ShooterState ss) {
    shooterSub = s;
    state = ss;
    addRequirements(shooterSub);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
        switch (state) {
            case Speaker:
                shooterSub.setSpeed(shooterSub.SpeakerVelocity);
                shooterSub.setAngle(shooterSub.SpeakerAngle);
                break;
            case Amp:
                shooterSub.setSpeed(shooterSub.AmpVelocity);
                shooterSub.setAngle(shooterSub.AmpAngle);
                break;
            case Source:
                shooterSub.setSpeed(shooterSub.SourceIntakeVelocity);
                shooterSub.setAngle(shooterSub.SourceIntakeAngle);
                break;
            case Chassis:
                shooterSub.setSpeed(shooterSub.ShooterFloorIntakeVelocity);
                shooterSub.setAngle(shooterSub.ShooterFloorIntakeAngle);
                break;
            case Trapdoor:
                // method goes here
                break;
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
