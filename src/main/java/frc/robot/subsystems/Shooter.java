// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElectronicIDs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sim.PhysicsSim;

public class Shooter extends SubsystemBase {
    TalonFX leftShooterMotor;
    TalonFX rightShooterMotor;

    DigitalInput breakBeam;
    DIOSim breakBeamSim;

    private final VelocityVoltage voltageVelocity = new VelocityVoltage(
        0, 0, true, 0, 0, 
        false, false, false);
    private final NeutralOut brake = new NeutralOut();

    public enum ShooterVelState {
        Stopped, Speaker, Amp, IntakeFromSource, IntakeFromFloor, Trap
    }

    public ShooterVelState shooterVelState = ShooterVelState.Stopped;

    boolean simulationInitialized = false;

    public Shooter() {
        leftShooterMotor = new TalonFX(ElectronicIDs.LeftShooterMotorID); // slot 0
        rightShooterMotor = new TalonFX(ElectronicIDs.RightShooterMotorID); // slot 1

        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

        configs.Slot1.kP = 0.11; // An error of 1 rotation per second results in 2V output
        configs.Slot1.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot1.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot1.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

        configs.Voltage.PeakForwardVoltage = 8; // Peak output of 8 volts
        configs.Voltage.PeakReverseVoltage = -8;

        breakBeam = new DigitalInput(ElectronicIDs.breakBeamID);
    }

    @Override
    public void periodic() {
    }

    public void setVelocity(double rotsPerSecond) {
        leftShooterMotor.setControl(voltageVelocity.withVelocity(rotsPerSecond));
        rightShooterMotor.setControl(voltageVelocity.withVelocity(rotsPerSecond));

        NetworkTableInstance.getDefault().getEntry("shooter/Desired Rotations Per Second").setDouble(rotsPerSecond);
    }

    public double getShooterVelocity() {
        return leftShooterMotor.getVelocity().getValue();
    }

    // private StatusSignal<Double> getFlyWheelClosedLoopError() {
    //     return leftShooterMotor.getClosedLoopError();
    // }

    public boolean isSpeakerShooting() {
        return getShooterVelocity() >= .95 * ShooterConstants.SpeakerVelocity;
    }

    public boolean isAmpShooting() {
        return (getShooterVelocity() >= .95 * ShooterConstants.AmpVelocity) &&
                (getShooterVelocity() <= 1.05 * ShooterConstants.AmpVelocity);
    }

    public boolean isSourceIntaking() {
        return (getShooterVelocity() >= .95 * ShooterConstants.SourceIntakeVelocity)
                && (getShooterVelocity() <= 1.05 * ShooterConstants.SourceIntakeVelocity);
    }

    public boolean isShooterFloorIntaking() {
        return (getShooterVelocity() >= .95 * ShooterConstants.ShooterFloorIntakeVelocity)
                && (getShooterVelocity() <= 1.05 * ShooterConstants.ShooterFloorIntakeVelocity);
    }

    public boolean isNoteLoaded() {
        // return breakBeam.get();
        return false; 
    }

    public String getShooterVelState() {
        return shooterVelState.toString();
    }

    /* LOGGING */

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter Subsystem");
        builder.addStringProperty("State", this::getShooterVelState, null);
        builder.addDoubleProperty("Actual Shooter Velocity", this::getShooterVelocity, null);
        builder.addBooleanProperty("Speaker Shooting", this::isSpeakerShooting, null);
        builder.addBooleanProperty("Amp Shooting", this::isAmpShooting, null);
        builder.addBooleanProperty("Source Intaking", this::isSourceIntaking, null);
        builder.addBooleanProperty("Floor Intaking", this::isShooterFloorIntaking, null);
        // builder.addBooleanProperty("Note Loaded", this::isNoteLoaded, null);
    }

    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(leftShooterMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(rightShooterMotor, 0.001);
        breakBeamSim = new DIOSim(breakBeam);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }
    }

}
