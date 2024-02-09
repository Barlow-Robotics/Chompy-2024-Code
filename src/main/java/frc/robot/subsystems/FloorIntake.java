// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.StatusCode;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FloorIntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sim.PhysicsSim;
import frc.robot.Constants;
import frc.robot.Constants.ElectronicsIDs;
import org.littletonrobotics.junction.Logger;

public class FloorIntake extends SubsystemBase {

    TalonFX intakeMotor;
    private final TalonFXSimState intakeMotorSim;
    private final DCMotorSim intakeMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1),
            1, ShooterConstants.jKgMetersSquared);

    private final VelocityVoltage voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0,
            false, false, false);
    private final NeutralOut brake = new NeutralOut();

    boolean simulationInitialized = false;

    public FloorIntake() {
        intakeMotor = new TalonFX(ElectronicsIDs.FloorMotorID);
        configMotor(false); // CHANGE - These true/false values may need to be flipped

        intakeMotorSim = intakeMotor.getSimState();
    }

    @Override
    public void periodic() {
        logData();
    }

    public void start() {
        intakeMotor.setControl(voltageVelocity.withVelocity(FloorIntakeConstants.MotorRPM / 60));
    }

    public void stop() {
        intakeMotor.setControl(brake);
    }

    public boolean isIntaking() {
        return intakeMotor.getVelocity().getValue() >= Constants.LowerToleranceLimit * FloorIntakeConstants.MotorRPM
                / 60;
    }

    /* LOGGING */

    private void logData() {
        Logger.recordOutput("FloorIntake/ActualRPM", intakeMotor.getVelocity().getValue());
        Logger.recordOutput("FloorIntake/IsIntaking", isIntaking());
    }

    /* CONFIG */
    private void configMotor(boolean inverted) {

        // Configure the current limits
        /* enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s) */
        // SupplyCurrentLimitConfiguration currentLimitsConfigs = new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5);
        // intakeMotor.configSupplyCurrentLimit(currentLimitsConfigs);

        // set PID Values
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        motorConfigs.Slot0.kP = FloorIntakeConstants.KP;
        motorConfigs.Slot0.kI = FloorIntakeConstants.KI;
        motorConfigs.Slot0.kD = FloorIntakeConstants.KD;

        // set current limit
        CurrentLimitsConfigs currentLimitConfigs = motorConfigs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimit = FloorIntakeConstants.SupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true; // Start with stator limits off

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = intakeMotor.getConfigurator().apply(motorConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply motor output configs to intake motor, with error code: " + status.toString());
        }
    }

    /* SIMULATION */

    private void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(intakeMotor, 0.001);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        intakeMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double voltage = intakeMotorSim.getMotorVoltage();
        intakeMotorModel.setInputVoltage(voltage);
        intakeMotorModel.update(0.02);
        intakeMotorSim.setRotorVelocity(intakeMotorModel.getAngularVelocityRPM() / 60.0);
        intakeMotorSim.setRawRotorPosition(intakeMotorModel.getAngularPositionRotations());
    }

}
