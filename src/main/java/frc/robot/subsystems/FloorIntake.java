// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FloorIntakeConstants;
import frc.robot.sim.PhysicsSim;
import frc.robot.Constants;
import frc.robot.Constants.ElectronicsIDs;
import org.littletonrobotics.junction.Logger;

public class FloorIntake extends SubsystemBase {

    TalonFX intakeMotor;
    private final TalonFXSimState intakeMotorSim;
    private final DCMotorSim intakeMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1),
            1, Constants.jKgMetersSquared);

    private final VelocityVoltage voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0,
            false, false, false);
    private final NeutralOut brake = new NeutralOut();

    boolean simulationInitialized = false;

    public FloorIntake() {
        intakeMotor = new TalonFX(ElectronicsIDs.FloorMotorID);
        applyMotorConfigs(InvertedValue.Clockwise_Positive);

        intakeMotorSim = intakeMotor.getSimState();
    }

    @Override
    public void periodic() {
        logData();
    }

    public void start() {
        Logger.recordOutput("FloorIntake/RPMDesired", FloorIntakeConstants.MotorRPM);
        intakeMotor.setControl(voltageVelocity.withVelocity(FloorIntakeConstants.MotorRPM / 60));
    }

    public void reverse() {
        Logger.recordOutput("FloorIntake/RPMDesired", -FloorIntakeConstants.MotorRPM);
        intakeMotor.setControl(voltageVelocity.withVelocity(-FloorIntakeConstants.MotorRPM / 60));
    }

    public void stop() {
        intakeMotor.setControl(brake);
    }

    public boolean isIntaking() {
        return intakeMotor.getVelocity().getValue() >= (FloorIntakeConstants.MotorRPM / 60) - FloorIntakeConstants.VelocityTolerance;
    }

    /* LOGGING */

    private void logData() {
        Logger.recordOutput("FloorIntake/RPMActual", intakeMotor.getVelocity().getValue() * 60);
        Logger.recordOutput("FloorIntake/IsIntaking", isIntaking());
        Logger.recordOutput("FloorIntake/CurrentSupply", intakeMotor.getSupplyCurrent().getValue());
    }

    /* CONFIG */

    private void applyMotorConfigs(InvertedValue inverted) {
        // set PID Values
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        motorConfigs.Slot0.kV = FloorIntakeConstants.FF;
        motorConfigs.Slot0.kP = FloorIntakeConstants.KP;
        motorConfigs.Slot0.kI = FloorIntakeConstants.KI;
        motorConfigs.Slot0.kD = FloorIntakeConstants.KD;

        MotorOutputConfigs invertConfigs = new MotorOutputConfigs();
        invertConfigs.Inverted = inverted;

        // set current limit
        CurrentLimitsConfigs currentLimitConfigs = motorConfigs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimit = FloorIntakeConstants.SupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true; // Start with stator limits off

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        // Try five times to apply the Intake motor config
        for (int i = 0; i < 5; ++i) {
            status = intakeMotor.getConfigurator().apply(motorConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply motor output configs to intake motor, with error code: " + status.toString());
        }

        // Try five times to apply the Intake motor invert config
        for (int i = 0; i < 5; ++i) {
            status = intakeMotor.getConfigurator().apply(invertConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply invert configs to intake motor, with error code: " + status.toString());
        }

        // Try five times to apply the Intake motor current config
        for (int i = 0; i < 5; ++i) {
            status = intakeMotor.getConfigurator().apply(currentLimitConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply current limit configs to intake motor, with error code: " + status.toString());
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
