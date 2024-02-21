// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sim.PhysicsSim;

public class Shooter extends SubsystemBase {
    private final TalonFX leftFlywheelMotor;
    private final TalonFXSimState leftFlywheelMotorSim;
    private final DCMotorSim lowerMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1),
            1, Constants.jKgMetersSquared);

    private final TalonFX rightFlywheelMotor;
    private final TalonFXSimState rightFlywheelMotorSim;
    private final DCMotorSim upperMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1),
            1, Constants.jKgMetersSquared);

    private final TalonFX indexMotor;
    private final TalonFXSimState indexMotorSim;
    private final DCMotorSim indexMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1),
            1, Constants.jKgMetersSquared);

    private final VelocityVoltage voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0,
            false, false, false);
    private final NeutralOut brake = new NeutralOut();

    DigitalInput breakBeam;
    DIOSim breakBeamSim;

    private boolean simulationInitialized = false;

    // private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    // private GenericEntry shuffleBoardSpeed = tab.add("ShuffleBoard Speed",
    // 1).getEntry();

    public Shooter() {
        leftFlywheelMotor = new TalonFX(ElectronicsIDs.LeftShooterMotorID);
        rightFlywheelMotor = new TalonFX(ElectronicsIDs.RightShooterMotorID);
        indexMotor = new TalonFX(ElectronicsIDs.IndexMotorID);

        applyLeftFlywheelMotorConfig(InvertedValue.Clockwise_Positive);
        applyRightFlywheelMotorConfig(InvertedValue.Clockwise_Positive);
        applyIndexMotorConfig(InvertedValue.Clockwise_Positive);

        leftFlywheelMotorSim = leftFlywheelMotor.getSimState();
        rightFlywheelMotorSim = rightFlywheelMotor.getSimState();
        indexMotorSim = indexMotor.getSimState();

        breakBeam = new DigitalInput(ElectronicsIDs.BreakBeamID);
    }

    @Override
    public void periodic() {
        logData();
    }

    public void startFlywheels(double shooterLeftRPM, double shooterRightRPM) {
        Logger.recordOutput("Shooter/DesiredMotorLeftRPM", shooterLeftRPM);
        Logger.recordOutput("Shooter/DesiredFlywheelLeftRPM", shooterLeftRPM * ShooterConstants.FlywheelGearRatio);
        Logger.recordOutput("Shooter/DesiredMotorRightRPM", shooterRightRPM);
        Logger.recordOutput("Shooter/DesiredFlywheelRightRPM", shooterRightRPM * ShooterConstants.FlywheelGearRatio);

        double shooterLeftRPS = shooterLeftRPM / Constants.SecondsPerMinute;
        double shooterRightRPS = shooterRightRPM / Constants.SecondsPerMinute;

        leftFlywheelMotor.setControl(voltageVelocity.withVelocity(shooterLeftRPS));
        rightFlywheelMotor.setControl(voltageVelocity.withVelocity(shooterRightRPS));

        Logger.recordOutput("Shooter/WithinToleranceFlywheels", isWithinFlywheelVelocityTolerance(shooterLeftRPM, shooterRightRPM));
    }

    public void startIndex(double indexRPM) {
        Logger.recordOutput("Shooter/DesiredIndexRPM", indexRPM);

        double indexRPS = indexRPM / Constants.SecondsPerMinute;

        indexMotor.setControl(voltageVelocity.withVelocity(indexRPS));
    }

    public void stop() {
        leftFlywheelMotor.setControl(voltageVelocity.withVelocity(0));
        rightFlywheelMotor.setControl(voltageVelocity.withVelocity(0));
        indexMotor.setControl(voltageVelocity.withVelocity(0));
    }

    private double getRPM(TalonFX motor) {
        return motor.getVelocity().getValue() * Constants.SecondsPerMinute;
    }

    public boolean isWithinFlywheelVelocityTolerance(double desiredLeftRPM, double desiredRightRPM) {
        return (getRPM(leftFlywheelMotor) >= desiredLeftRPM - ShooterConstants.VelocityTolerance) &&
                (getRPM(rightFlywheelMotor) >= desiredRightRPM - ShooterConstants.VelocityTolerance);
    }

    public boolean isNoteLoaded() {
        return !breakBeam.get();
    }

    /* LOGGING */

    private void logData() {
        Logger.recordOutput("Shooter/ActualRPMMotorLeft", getRPM(leftFlywheelMotor));
        Logger.recordOutput("Shooter/ActualRPMMotorRight", getRPM(rightFlywheelMotor));
        Logger.recordOutput("Shooter/ActualRPMFlywheelRight", getRPM(leftFlywheelMotor)
                * ShooterConstants.FlywheelGearRatio);
        Logger.recordOutput("Shooter/ActualRPMFlywheelRight", getRPM(rightFlywheelMotor)
                * ShooterConstants.FlywheelGearRatio);
        Logger.recordOutput("Shooter/ActualRPMIndex", getRPM(indexMotor));
        Logger.recordOutput("Shooter/ClosedLoopError/Lower", leftFlywheelMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Shooter/ClosedLoopError/Upper", rightFlywheelMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Shooter/IsNoteLoaded", isNoteLoaded());
        Logger.recordOutput("Shooter/SupplyCurrent/FlywheelLower", leftFlywheelMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Shooter/SupplyCurrent/FlywheelUpper", rightFlywheelMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Shooter/SupplyCurrent/Index", indexMotor.getSupplyCurrent().getValue());
    }

    /* CONFIG */

    private void applyIndexMotorConfig(InvertedValue inversion) {
        TalonFXConfiguration PIDConfigs = new TalonFXConfiguration();
        PIDConfigs.Slot0.kP = ShooterConstants.IndexKP;
        PIDConfigs.Slot0.kI = ShooterConstants.IndexKI;
        PIDConfigs.Slot0.kD = ShooterConstants.IndexKD;
        PIDConfigs.Slot0.kV = ShooterConstants.IndexFF;

        applyMotorConfig(indexMotor, "indexMotor", PIDConfigs, inversion);
    }

    private void applyLeftFlywheelMotorConfig(InvertedValue inversion) {
        TalonFXConfiguration PIDConfigs = new TalonFXConfiguration();
        PIDConfigs.Slot0.kP = ShooterConstants.FlywheelLeftKP;
        PIDConfigs.Slot0.kI = ShooterConstants.FlywheelLeftKI;
        PIDConfigs.Slot0.kD = ShooterConstants.FlywheelLeftKD;
        PIDConfigs.Slot0.kV = ShooterConstants.FlywheelLeftFF;

        applyMotorConfig(leftFlywheelMotor, "leftFlywheelMotor", PIDConfigs, inversion);
    }

    private void applyRightFlywheelMotorConfig(InvertedValue inversion) {
        TalonFXConfiguration PIDConfigs = new TalonFXConfiguration();
        PIDConfigs.Slot0.kP = ShooterConstants.FlywheelRightKP;
        PIDConfigs.Slot0.kI = ShooterConstants.FlywheelRightKI;
        PIDConfigs.Slot0.kD = ShooterConstants.FlywheelRightKD;
        PIDConfigs.Slot0.kV = ShooterConstants.FlywheelRightFF;

        applyMotorConfig(rightFlywheelMotor, "rightFlywheelMotor", PIDConfigs, inversion);
    }

    private void applyMotorConfig(TalonFX motor, String motorName, TalonFXConfiguration PIDConfigs, InvertedValue inversion) {

        /* SET TO COAST MODE */
        motor.setControl(brake);

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        /* APPLY PID */

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(PIDConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply PID configs to " + motorName + ", error code: " + status.toString());
        }

        /* SET & APPLY INVERSION */

        motorOutputConfigs.Inverted = inversion;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(motorOutputConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply inversion configs to " + motorName + ", error code: " + status.toString());
        }

        /* SET & APPLY CURRENT LIMIT */

        CurrentLimitsConfigs currentLimitConfigs = PIDConfigs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(currentLimitConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply current limit configs to " + motorName + ", error code: " + status.toString());
        }
    }

    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(leftFlywheelMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(rightFlywheelMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(indexMotor, 0.001);

        leftFlywheelMotorSim.Orientation = ChassisReference.CounterClockwise_Positive; // CHANGE
        rightFlywheelMotorSim.Orientation = ChassisReference.Clockwise_Positive; // CHANGE

        breakBeamSim = new DIOSim(breakBeam);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        leftFlywheelMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double lowerVoltage = leftFlywheelMotorSim.getMotorVoltage();
        lowerMotorModel.setInputVoltage(lowerVoltage);
        lowerMotorModel.update(0.02);
        leftFlywheelMotorSim.setRotorVelocity(lowerMotorModel.getAngularVelocityRPM() / 60.0);
        leftFlywheelMotorSim.setRawRotorPosition(lowerMotorModel.getAngularPositionRotations());

        rightFlywheelMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double upperVoltage = rightFlywheelMotorSim.getMotorVoltage();
        upperMotorModel.setInputVoltage(upperVoltage);
        upperMotorModel.update(0.02);
        rightFlywheelMotorSim.setRotorVelocity(upperMotorModel.getAngularVelocityRPM() / 60.0);
        rightFlywheelMotorSim.setRawRotorPosition(upperMotorModel.getAngularPositionRotations());

        indexMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double indexVoltage = indexMotorSim.getMotorVoltage();
        indexMotorModel.setInputVoltage(indexVoltage);
        indexMotorModel.update(0.02);
        indexMotorSim.setRotorVelocity(indexMotorModel.getAngularVelocityRPM() / 60.0);
        indexMotorSim.setRawRotorPosition(indexMotorModel.getAngularPositionRotations());
    }
}