// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElectronicIDs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sim.PhysicsSim;

public class Shooter extends SubsystemBase {
    TalonFX leftShooterMotor;
    TalonFX rightShooterMotor;

    private final TalonFXSimState leftShooterMotorSim;
    private final DCMotorSim leftMotorModel = 
        new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1, 0.0005);
    
    private final TalonFXSimState rightShooterMotorSim;
    private final DCMotorSim rightMotorModel = 
        new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1, 0.0005);

    private final VelocityVoltage voltageVelocity = new VelocityVoltage(
        0, 0, true, 0, 0,
        false, false, false);
    private final NeutralOut brake = new NeutralOut();

    public enum ShooterVelState {
        Stopped, Speaker, Amp, IntakeFromSource, IntakeFromFloor, Trap
    }

    public ShooterVelState shooterVelState = ShooterVelState.Stopped;

    DigitalInput breakBeam;
    DIOSim breakBeamSim;

    boolean simulationInitialized = false;

    public Shooter() {
        leftShooterMotor = new TalonFX(ElectronicIDs.LeftShooterMotorID); // slot 0
        rightShooterMotor = new TalonFX(ElectronicIDs.RightShooterMotorID); // slot 0

        leftShooterMotorSim = leftShooterMotor.getSimState();
        rightShooterMotorSim = rightShooterMotor.getSimState();

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
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

    public double getLeftShooterVelocity() {
        return leftShooterMotor.getVelocity().getValue();
    }

    public double getRightShooterVelocity() {
        return rightShooterMotor.getVelocity().getValue();
    }

    private double getLeftShooterClosedLoopError() {
        return leftShooterMotor.getClosedLoopError().getValue();
    }

    private double getRightShooterClosedLoopError() {
        return rightShooterMotor.getClosedLoopError().getValue();
    }

    public boolean isSpeakerShooting() {
        return getLeftShooterVelocity() >= .95 * ShooterConstants.SpeakerVelocity;
    }

    public boolean isAmpShooting() {
        return (getLeftShooterVelocity() >= .95 * ShooterConstants.AmpVelocity) &&
                (getLeftShooterVelocity() <= 1.05 * ShooterConstants.AmpVelocity);
    }

    public boolean isSourceIntaking() {
        return (getLeftShooterVelocity() >= .95 * ShooterConstants.SourceIntakeVelocity)
                && (getLeftShooterVelocity() <= 1.05 * ShooterConstants.SourceIntakeVelocity);
    }

    public boolean isShooterFloorIntaking() {
        return (getLeftShooterVelocity() >= .95 * ShooterConstants.ShooterFloorIntakeVelocity)
                && (getLeftShooterVelocity() <= 1.05 * ShooterConstants.ShooterFloorIntakeVelocity);
    }
    public boolean isTrapShooting() {
        return (getLeftShooterVelocity() >= .95 * ShooterConstants.TrapVelocity)
                && (getLeftShooterVelocity() <= 1.05 * ShooterConstants.TrapVelocity);
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
        builder.addDoubleProperty("Actual Left Shooter Velocity", this::getLeftShooterVelocity, null);
        builder.addDoubleProperty("Actual Right Shooter Velocity", this::getRightShooterVelocity, null);
        builder.addDoubleProperty("Left Shooter Closed Loop Error", this::getLeftShooterClosedLoopError, null);
        builder.addDoubleProperty("Right Shooter Closed Loop Error", this::getRightShooterClosedLoopError, null);
        builder.addBooleanProperty("Speaker Shooting", this::isSpeakerShooting, null);
        builder.addBooleanProperty("Amp Shooting", this::isAmpShooting, null);
        builder.addBooleanProperty("Source Intaking", this::isSourceIntaking, null);
        builder.addBooleanProperty("Floor Intaking", this::isShooterFloorIntaking, null);
        builder.addBooleanProperty("Trap Shooting", this::isTrapShooting, null);
        // builder.addBooleanProperty("Note Loaded", this::isNoteLoaded, null);
    }

    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(leftShooterMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(rightShooterMotor, 0.001);

        leftShooterMotorSim.Orientation = ChassisReference.CounterClockwise_Positive; // CHANGE
        rightShooterMotorSim.Orientation = ChassisReference.Clockwise_Positive; // CHANGE

        breakBeamSim = new DIOSim(breakBeam);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        leftShooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double leftVoltage = leftShooterMotorSim.getMotorVoltage();
        leftMotorModel.setInputVoltage(leftVoltage);
        leftMotorModel.update(0.02);
        leftShooterMotorSim.setRotorVelocity(leftMotorModel.getAngularVelocityRPM() / 60.0);
        leftShooterMotorSim.setRawRotorPosition(leftMotorModel.getAngularPositionRotations());

        rightShooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double rightVoltage = rightShooterMotorSim.getMotorVoltage();
        rightMotorModel.setInputVoltage(rightVoltage);
        rightMotorModel.update(0.02);
        rightShooterMotorSim.setRotorVelocity(rightMotorModel.getAngularVelocityRPM() / 60.0);
        rightShooterMotorSim.setRawRotorPosition(rightMotorModel.getAngularPositionRotations());
    }

}
