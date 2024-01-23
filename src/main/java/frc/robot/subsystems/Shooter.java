// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CanIDs;
import frc.robot.sim.PhysicsSim;

public class Shooter extends SubsystemBase {
    /*********************************************************************/
    /***************************** CONSTANTS *****************************/

    public static final double SpeakerVelocity = 0; // CHANGE
    public static final double AmpVelocity = 0; // CHANGE
    public static final double SourceIntakeVelocity = 0; // CHANGE
    public static final double ShooterFloorIntakeVelocity = 0; // CHANGE

    public static final double SpeakerAngle = 0; // CHANGE
    public static final double AmpAngle = 0; // CHANGE
    public static final double SourceIntakeAngle = 0; // CHANGE
    public static final double ShooterFloorIntakeAngle = 0; // CHANGE

    boolean simulationInitialized = false;
    private static final int simulationVelocity = 6800; // CHANGE
    private static final double simulationTime = 0.5; // CHANGE

    /*********************************************************************/
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

    public enum ShooterState {
        Stopped, Speaker, Amp, Source, Chassis, Trapdoor
    }

    public ShooterState shooterState = ShooterState.Stopped;

    @Override
    public void periodic() {

    }

    public void setSpeed(double velocity) {
        leftShooterMotor.set(velocity); // .set() uses % output, not veolocity (need to find a new function)
        rightShooterMotor.set(velocity);
    }

    public double getShooterVelocity() {
        return leftShooterMotor.getVelocity().getValue();
    }

    public boolean isSpeakerShooting() {
        return getShooterVelocity() >= .95 * SpeakerVelocity;
    }

    public boolean isAmpShooting() {
        return (getShooterVelocity() >= .95 * AmpVelocity) && (getShooterVelocity() <= 1.05 * AmpVelocity);
    }

    public boolean isSourceIntaking() {
        return (getShooterVelocity() >= .95 * SourceIntakeVelocity)
                && (getShooterVelocity() <= 1.05 * SourceIntakeVelocity);
    }

    public boolean isShooterFloorIntaking() {
        return (getShooterVelocity() >= .95 * ShooterFloorIntakeVelocity)
                && (getShooterVelocity() <= 1.05 * ShooterFloorIntakeVelocity);
    }

    public boolean isNoteLoaded() {
        return (breakBeam.get());
    }

    public void setAngle(double angle) {
        angleMotor.setPosition(angle);
    }

    public double getAngle() {
        return angleMotor.getPosition().getValue();
    }

    public String getShooterState() {
        return shooterState.toString();
    }

    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("State", this::getShooterState, null);
        builder.addDoubleProperty("Shooter Velocity", this::getShooterVelocity, null);
        builder.addBooleanProperty("Speaker Shooting", this::isSpeakerShooting, null);
        builder.addBooleanProperty("Amp Shooting", this::isAmpShooting, null);
        builder.addBooleanProperty("Source Intaking", this::isSourceIntaking, null);
        builder.addBooleanProperty("Floor Intaking", this::isShooterFloorIntaking, null);
        builder.addBooleanProperty("Note Loaded", this::isNoteLoaded, null);
        builder.addDoubleProperty("Get Angle", this::getAngle, null);

        // builder.addStringProperty()
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
