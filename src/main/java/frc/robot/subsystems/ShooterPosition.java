// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterPositionConstants;
import frc.robot.sim.PhysicsSim;

import frc.robot.Constants;

public class ShooterPosition extends SubsystemBase {

    TalonFX angleMotor;
    private final TalonFXSimState angleMotorSim;
    private final DCMotorSim angleMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1),
            1, 0.0005);

    TalonFX leftElevatorMotor;
    private final TalonFXSimState leftElevatorMotorSim;
    private final DCMotorSim leftElevatorMotorModel = new DCMotorSim(
            edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1, 0.0005);

    TalonFX rightElevatorMotor;
    private final TalonFXSimState rightElevatorMotorSim;
    private final DCMotorSim rightElevatorMotorModel = new DCMotorSim(
            edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1, 0.0005);

    private final VelocityVoltage angleVoltageVelocity = 
        new VelocityVoltage(0, 0, true, 0, 0, 
                            false, false, false);
    private final VelocityVoltage elevatorVoltageVelocity = 
        new VelocityVoltage(0, 0, true, 0, 1, 
                            false, false, false);
    private final NeutralOut brake = new NeutralOut();

    DigitalInput bottomHallEffect;
    DigitalInput topHallEffect;

    public enum ShooterPositionState {
        Speaker, Amp, SourceIntake, FloorIntake, Trap, MovingToPosition
    }

    public ShooterPositionState shooterPosState = ShooterPositionState.FloorIntake;

    private boolean simulationInitialized = false;

    public ShooterPosition() {
        angleMotor = new TalonFX(ElectronicsIDs.AngleMotorID);
        angleMotorSim = angleMotor.getSimState();

        leftElevatorMotor = new TalonFX(ElectronicsIDs.LeftElevatorMotorID);
        leftElevatorMotorSim = leftElevatorMotor.getSimState();

        rightElevatorMotor = new TalonFX(ElectronicsIDs.RightElevatorMotorID);
        rightElevatorMotorSim = rightElevatorMotor.getSimState();
        rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), true));

        TalonFXConfiguration configs = new TalonFXConfiguration();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        setTalonConfigs(configs);
        applyMotorConfigs(angleMotor, "angleMotor", configs, motorOutputConfigs, InvertedValue.Clockwise_Positive); // CHANGE
        applyMotorConfigs(leftElevatorMotor, "leftElevatorMotor", configs, motorOutputConfigs, InvertedValue.Clockwise_Positive); // CHANGE
        applyMotorConfigs(rightElevatorMotor, "rightElevatorMotor", configs, motorOutputConfigs, InvertedValue.CounterClockwise_Positive); // CHANGE

        bottomHallEffect = new DigitalInput(ElectronicsIDs.BottomHallEffectID);
        topHallEffect = new DigitalInput(ElectronicsIDs.TopHallEffectID);
    }

    @Override
    public void periodic() {
        advantageKitLogging();
    }

    /** @param angle Desired angle in fraction of a rotation */ // May want to CHANGE this to degrees
    public void setAngle(double angle) {
        angleMotor.setPosition(angle);
        Logger.recordOutput("ShooterPosition/AngleDesired", angle);
    }

    public double getAngle() {
        return angleMotor.getPosition().getValue();
    }

    public void setHeight(double height) {
        leftElevatorMotor.setPosition(height * ShooterPositionConstants.RotationsPerElevatorInch);
    }

    public double getHeight() {
        return leftElevatorMotor.getPosition().getValueAsDouble() / ShooterPositionConstants.RotationsPerElevatorInch;
    }


    public void setShooterPosState(ShooterPositionState newState) {
        shooterPosState = newState;
    }

    public ShooterPositionState getShooterPosState() {
        return shooterPosState;
    }

    public String getShooterPosStateAsString() {
        return shooterPosState.toString();
    }

    private boolean isWithinAngleTolerance(double desiredAngle) {
        return (getAngle() >= Constants.LowerToleranceLimit * desiredAngle) &&
                (getAngle() <= Constants.UpperToleranceLimit * desiredAngle);
    }

    private boolean isWithinHeightTolerance(double desiredHeight) {
        return (getHeight() >= Constants.LowerToleranceLimit * desiredHeight) &&
                (getHeight() <= Constants.UpperToleranceLimit * desiredHeight);
    }

    public boolean isWithinPositionTolerance(double desiredAngle, double desiredHeight) {
        return (getAngle() >= Constants.LowerToleranceLimit * desiredAngle) &&
                (getAngle() <= Constants.UpperToleranceLimit * desiredAngle) &&
                (getHeight() >= Constants.LowerToleranceLimit * desiredHeight) &&
                (getHeight() <= Constants.UpperToleranceLimit * desiredHeight);
    }

    public boolean isAtBottom() {
        return !bottomHallEffect.get(); // might need to get rid of the ! depending on how the hall effect works
    }

    public boolean isAtTop() {
        return !topHallEffect.get();
    }

    private void advantageKitLogging() {
        Logger.recordOutput("ShooterPosition/State", getShooterPosStateAsString());
        Logger.recordOutput("ShooterPosition/AngleActual", getAngle());
        Logger.recordOutput("ShooterPosition/IsAtSpeakerAngle", isWithinAngleTolerance(ShooterPositionConstants.SpeakerAngle));
        Logger.recordOutput("ShooterPosition/IsAtSpeakerHeight", isWithinHeightTolerance(ShooterPositionConstants.SpeakerHeight));
        Logger.recordOutput("ShooterPosition/IsAtAmpAngle", isWithinAngleTolerance(ShooterPositionConstants.AmpAngle));
        Logger.recordOutput("ShooterPosition/IsAtAmpHeight", isWithinHeightTolerance(ShooterPositionConstants.AmpHeight));
        Logger.recordOutput("ShooterPosition/IsAtSourceAngle", isWithinAngleTolerance(ShooterPositionConstants.SourceIntakeAngle));
        Logger.recordOutput("ShooterPosition/IsAtSourceHeight", isWithinHeightTolerance(ShooterPositionConstants.SourceIntakeHeight));
        Logger.recordOutput("ShooterPosition/IsAtFloorAngle", isWithinAngleTolerance(ShooterPositionConstants.FloorIntakeAngle));
        Logger.recordOutput("ShooterPosition/IsAtFloorHeight", isWithinHeightTolerance(ShooterPositionConstants.FloorIntakeHeight));
        Logger.recordOutput("ShooterPosition/IsAtTrapAngle", isWithinAngleTolerance(ShooterPositionConstants.TrapAngle));
        Logger.recordOutput("ShooterPosition/IsAtTrapHeight", isWithinHeightTolerance(ShooterPositionConstants.TrapHeight));
        Logger.recordOutput("ShooterPosition/IsAtBottom", isAtBottom());
        Logger.recordOutput("ShooterPosition/IsAtTop", isAtTop());
    }

    /* CONFIG */

    private void setTalonConfigs(TalonFXConfiguration configs) {
        configs.Slot0.kP = ShooterPositionConstants.AngleKP;
        configs.Slot0.kI = ShooterPositionConstants.AngleKI;
        configs.Slot0.kD = ShooterPositionConstants.AngleKD;
        configs.Slot0.kV = ShooterPositionConstants.AngleFF;

        configs.Slot1.kP = ShooterPositionConstants.ElevatorKP;
        configs.Slot1.kI = ShooterPositionConstants.ElevatorKI;
        configs.Slot1.kD = ShooterPositionConstants.ElevatorKD;
        configs.Slot1.kV = ShooterPositionConstants.ElevatorFF;
       
        configs.Voltage.PeakForwardVoltage = ShooterConstants.PeakShooterForwardVoltage; // Peak output of 8 volts
        configs.Voltage.PeakReverseVoltage = ShooterConstants.PeakShooterReverseVoltage;
    }

    private void applyMotorConfigs(
            TalonFX motor, String motorName,
            TalonFXConfiguration talonConfigs, MotorOutputConfigs motorOutputConfigs, InvertedValue inversion) {

        motorOutputConfigs.Inverted = inversion;

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        // attempt setting configs up to 5 times
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(talonConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply talon configs to " + motorName + " error code: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(motorOutputConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply motor output configs to " + motorName + " error code: " + status.toString());
        }
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

        leftElevatorMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double leftVoltage = leftElevatorMotorSim.getMotorVoltage();
        leftElevatorMotorModel.setInputVoltage(leftVoltage);
        leftElevatorMotorModel.update(0.02);
        leftElevatorMotorSim.setRotorVelocity(leftElevatorMotorModel.getAngularVelocityRPM() / 60.0);
        leftElevatorMotorSim.setRawRotorPosition(leftElevatorMotorModel.getAngularPositionRotations());

        rightElevatorMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double rightVoltage = rightElevatorMotorSim.getMotorVoltage();
        rightElevatorMotorModel.setInputVoltage(rightVoltage);
        rightElevatorMotorModel.update(0.02);
        rightElevatorMotorSim.setRotorVelocity(rightElevatorMotorModel.getAngularVelocityRPM() / 60.0);
        rightElevatorMotorSim.setRawRotorPosition(rightElevatorMotorModel.getAngularPositionRotations());
    }
}