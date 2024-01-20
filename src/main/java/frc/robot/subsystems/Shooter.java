// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.CanIDs;
import frc.robot.sim.PhysicsSim;

public class Shooter extends SubsystemBase {
    /***************************** CONSTANTS *****************************/

    private static final double ShooterVelocity = 0; // CHANGE
    private static final double ShooterAsIntakeVelocity = 0; // CHANGE

    boolean simulationInitialized = false;
    private static final int simulationVelocity = 6800; // CHANGE
    private static final double simulationTime = 0.5; // CHANGE
    /*********************************************************************/

    TalonFX leftShooterMotor;
    TalonFX rightShooterMotor;
    TalonFX angleMotor;
    DigitalInput breakBeam;

    public Shooter() {
        leftShooterMotor = new TalonFX(CanIDs.LeftShooterMotorID);
        rightShooterMotor = new TalonFX(CanIDs.RightShooterMotorID);
        angleMotor = new TalonFX(CanIDs.AngleMotorID);
    }

    public void startShooter() {
        leftShooterMotor.set(ShooterVelocity);
        rightShooterMotor.set(ShooterVelocity);
    }

    public void startShooterAsIntake() {
      // any different behavior required if intaking from source vs floor??? 
        leftShooterMotor.set(ShooterAsIntakeVelocity);
        rightShooterMotor.set(ShooterAsIntakeVelocity);
    }

    public void stopShooter() {
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
    }

    public double getShooterVelocity() {
        return leftShooterMotor.getVelocity().getValue();
    }

    public boolean isShooting() {
        return getShooterVelocity() >= .95 * ShooterVelocity;
    }

    public boolean isNoteLoaded() {
        return (breakBeam.get());
    }

    public double getAngle() {
        return angleMotor.getPosition().getValue();
    }

    public void setAngle(int angle) {
        angleMotor.setPosition(angle);
    }

    @Override
    public void periodic() {

    }

    // Simulation Code
    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(leftShooterMotor, simulationTime, simulationVelocity);
        PhysicsSim.getInstance().addTalonFX(rightShooterMotor, simulationTime, simulationVelocity);
        PhysicsSim.getInstance().addTalonFX(angleMotor, simulationTime, simulationVelocity);
      }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }
    }
}
