// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectronicIDs;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sim.PhysicsSim;

public class ShooterPosition extends SubsystemBase {

    DigitalInput bottomHallEffect;

    TalonFX angleMotor;
    TalonFX leftElevatorMotor;
    TalonFX rightElevatorMotor;

    boolean simulationInitialized = false;
    private final TalonFXSimState angleMotorSim;
    private final DCMotorSim angleMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1),
            1, 0.0005);
    private final TalonFXSimState leftMotorSim;
    private final DCMotorSim leftMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1),
            1, 0.0005);
    private final TalonFXSimState rightMotorSim;
    private final DCMotorSim rightMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1),
            1, 0.0005);

    public enum ShooterPositionState {
        Speaker, Amp, IntakeFromSource, IntakeFromFloor, Trap
    }

    public ShooterPositionState shooterPosState = ShooterPositionState.IntakeFromFloor;

    public ShooterPosition() {
        angleMotor = new TalonFX(ElectronicIDs.AngleMotorID);

        angleMotorSim = angleMotor.getSimState();
        leftMotorSim = leftElevatorMotor.getSimState();
        rightMotorSim = rightElevatorMotor.getSimState();

        leftElevatorMotor = new TalonFX(ElectronicIDs.LeftElevatorMotorID);
        rightElevatorMotor = new TalonFX(ElectronicIDs.RightElevatorMotorID);

        rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), true));

        bottomHallEffect = new DigitalInput(ElectronicIDs.HallEffectID);
    }

    @Override
    public void periodic() {
        if (getBottomHallEffect()) {
            setHeight(0);
        }
    }

    /** @param angle Desired angle in fraction of a rotation */ // May want to CHANGE this to degrees
    public void setAngle(double angle) {
        angleMotor.setPosition(angle);
    }

    public double getAngle() {
        return angleMotor.getPosition().getValue();
    }

    public ShooterPositionState getShooterPosState() {
        return shooterPosState;
    }

    public String getShooterPosStateAsString() {
        return shooterPosState.toString();
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

    public void setHeight(double height) {
        leftElevatorMotor.setPosition(height * ElevatorConstants.RotationsPerElevatorInch);
    }

    public double getHeight() {
        return leftElevatorMotor.getPosition().getValueAsDouble() / ElevatorConstants.RotationsPerElevatorInch;
    }

    public boolean getBottomHallEffect() {
        return !bottomHallEffect.get(); // might need to get rid of the ! depending on how the hall effect works
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter Angle Subsystem");
        builder.addStringProperty("State", this::getShooterPosStateAsString, null);
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addBooleanProperty("Speaker Angle", this::isSpeakerAngled, null);
        builder.addBooleanProperty("Amp Angle", this::isAmpAngled, null);
        builder.addBooleanProperty("Source Angle", this::isSourceAngled, null);
        builder.addBooleanProperty("Floor Angle", this::isFloorAngled, null);
        builder.addBooleanProperty("Trap Angle", this::isTrapAngled, null);
        builder.addDoubleProperty("Elevator Height", this::getHeight, null);
        builder.addBooleanProperty("Bottom Hall Effect", this::getBottomHallEffect, null);

        // builder.addBooleanProperty("Note Loaded", this::isNoteLoaded, null);
    }
    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(angleMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(leftElevatorMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(rightElevatorMotor, 0.001);
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

        leftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double leftVoltage = leftMotorSim.getMotorVoltage();
        leftMotorModel.setInputVoltage(leftVoltage);
        leftMotorModel.update(0.02);
        leftMotorSim.setRotorVelocity(leftMotorModel.getAngularVelocityRPM() / 60.0);
        leftMotorSim.setRawRotorPosition(leftMotorModel.getAngularPositionRotations());

        rightMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double rightVoltage = rightMotorSim.getMotorVoltage();
        rightMotorModel.setInputVoltage(rightVoltage);
        rightMotorModel.update(0.02);
        rightMotorSim.setRotorVelocity(rightMotorModel.getAngularVelocityRPM() / 60.0);
        rightMotorSim.setRawRotorPosition(rightMotorModel.getAngularPositionRotations());
    }
}