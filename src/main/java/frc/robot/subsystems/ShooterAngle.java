// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectronicIDs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sim.PhysicsSim;

public class ShooterAngle extends SubsystemBase {
    TalonFX angleMotor;
    boolean simulationInitialized = false;
    private final TalonFXSimState angleMotorSim;
    private final DCMotorSim angleMotorModel = 
        new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1, 0.0005);
    
    public enum ShooterPositionState {
        Speaker, Amp, IntakeFromSource, IntakeFromFloor, Trap
    }

    public ShooterPositionState shooterAngleState = ShooterPositionState.IntakeFromFloor;

    public ShooterAngle() {
        angleMotor = new TalonFX(ElectronicIDs.AngleMotorID);
        angleMotorSim = angleMotor.getSimState();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /** @param angle Desired angle in fraction of a rotation */ //May want to CHANGE this to degrees
    public void setAngle(double angle) {
        angleMotor.setPosition(angle);
    }

    public double getAngle() {
        return angleMotor.getPosition().getValue();
    }
    public String getShooterAngleState() {
        return shooterAngleState.toString();
    }
    public boolean isSpeakerAngled() {
        return (getAngle() >= .95 * ShooterConstants.SpeakerAngle)
                && (getAngle() <= 1.05 * ShooterConstants.SpeakerAngle);
    }
    public boolean isAmpAngled() {
        return (getAngle() >= .95 * ShooterConstants.AmpAngle)
                && (getAngle() <= 1.05 * ShooterConstants.AmpAngle);
    }
    public boolean isSourceAngled() {
        return (getAngle() >= .95 * ShooterConstants.IntakeFromSourceAngle)
                && (getAngle() <= 1.05 * ShooterConstants.IntakeFromSourceAngle);
    }
    public boolean isFloorAngled() {
        return (getAngle() >= .95 * ShooterConstants.IntakeFromFloorAngle)
                && (getAngle() <= 1.05 * ShooterConstants.IntakeFromFloorAngle);
    }
    public boolean isTrapAngled() {
        return (getAngle() >= .95 * ShooterConstants.TrapAngle)
                && (getAngle() <= 1.05 * ShooterConstants.TrapAngle);
    }
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter Angle Subsystem");
        builder.addStringProperty("State", this::getShooterAngleState, null);
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addBooleanProperty("Speaker Angle", this::isSpeakerAngled, null);
        builder.addBooleanProperty("Amp Angle", this::isAmpAngled, null);
        builder.addBooleanProperty("Source Angle", this::isSourceAngled, null);
        builder.addBooleanProperty("Floor Angle", this::isFloorAngled, null);
        builder.addBooleanProperty("Trap Angle", this::isTrapAngled, null);
        // builder.addBooleanProperty("Note Loaded", this::isNoteLoaded, null);
    }
     /* SIMULATION */

     public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(angleMotor, 0.001);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        angleMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double angleVoltage = angleMotorSim.getMotorVoltage();
        angleMotorModel.setInputVoltage(angleVoltage);
        angleMotorModel.update(0.02);
        angleMotorSim.setRotorVelocity(angleMotorModel.getAngularVelocityRPM() / 60.0);
        angleMotorSim.setRawRotorPosition(angleMotorModel.getAngularPositionRotations());
    }
}