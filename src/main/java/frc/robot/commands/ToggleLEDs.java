// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Underglow;

public class ToggleLEDs extends Command {

    Underglow underglowSub;

    int num = 3;

    public ToggleLEDs(Underglow underglowSub) {
        this.underglowSub = underglowSub;
        addRequirements(underglowSub);
    }

    @Override
    public void initialize() {
        if (num != 3) {
            num++;
        } else {
            num = 1;
        }
    }

    @Override
    public void execute() {
        if (num == 1) { // normal leds 
            underglowSub.LEDHumanFloor = false;
            underglowSub.LEDHumanSource = false;
        } else if (num == 2) { // source floor leds 
            underglowSub.LEDHumanFloor = true;
            underglowSub.LEDHumanSource = false;
        } else { // source source leds 
            underglowSub.LEDHumanFloor = false;
            underglowSub.LEDHumanSource = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
