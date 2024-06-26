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
    private final TalonFX topFlywheelMotor;
    private final TalonFXSimState leftFlywheelMotorSim;
    private final DCMotorSim lowerMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1),
            1, Constants.jKgMetersSquared);

    private final TalonFX bottomFlywheelMotor;
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

    public Shooter() {
        topFlywheelMotor = new TalonFX(ElectronicsIDs.TopShooterMotorID);
        bottomFlywheelMotor = new TalonFX(ElectronicsIDs.BottomShooterMotorID);
        indexMotor = new TalonFX(ElectronicsIDs.IndexMotorID);

        applyTopFlywheelMotorConfig(InvertedValue.Clockwise_Positive);
        applyBottomFlywheelMotorConfig(InvertedValue.Clockwise_Positive);
        applyIndexMotorConfig(InvertedValue.Clockwise_Positive);

        leftFlywheelMotorSim = topFlywheelMotor.getSimState();
        rightFlywheelMotorSim = bottomFlywheelMotor.getSimState();
        indexMotorSim = indexMotor.getSimState();

        breakBeam = new DigitalInput(ElectronicsIDs.BreakBeamID);
    }

    @Override
    public void periodic() {
        logData();
    }

    public void startFlywheels(double topRPM, double bottomRPM) {
        Logger.recordOutput("Shooter/Top/MotorDesiredRPM", topRPM);
        Logger.recordOutput("Shooter/Top/FlywheelDesiredRPM", topRPM * ShooterConstants.FlywheelGearRatio);
        Logger.recordOutput("Shooter/Bottom/MotorDesiredRPM", bottomRPM);
        Logger.recordOutput("Shooter/Bottom/FlywheelDesiredRPM", bottomRPM * ShooterConstants.FlywheelGearRatio);

        double topRPS = topRPM / Constants.SecondsPerMinute;
        double bottomRPS = bottomRPM / Constants.SecondsPerMinute;

        // we separate the two in the event that we wish to control them separately to modify the trajectory of the note 
        topFlywheelMotor.setControl(voltageVelocity.withVelocity(topRPS));
        bottomFlywheelMotor.setControl(voltageVelocity.withVelocity(bottomRPS));

        Logger.recordOutput("Shooter/WithinToleranceFlywheels", isWithinFlywheelVelocityTolerance(topRPM, bottomRPM));
    }

    public void startIndex(double indexRPM) {
        Logger.recordOutput("Shooter/Index/DesiredRPM", indexRPM);

        double indexRPS = indexRPM / Constants.SecondsPerMinute;

        indexMotor.setControl(voltageVelocity.withVelocity(indexRPS));
    }

    public void stop() {
        topFlywheelMotor.setControl(voltageVelocity.withVelocity(0));
        bottomFlywheelMotor.setControl(voltageVelocity.withVelocity(0));
        indexMotor.setControl(voltageVelocity.withVelocity(0));
    }

    private double getRPM(TalonFX motor) {
        return motor.getVelocity().getValue() * Constants.SecondsPerMinute;
    }

    public boolean isShooting() {
        return topFlywheelMotor.getVelocity().getValue() > 0;
    }

    public boolean isWithinFlywheelVelocityTolerance(double desiredLeftRPM, double desiredRightRPM) {
        return (getRPM(topFlywheelMotor) >= desiredLeftRPM - ShooterConstants.FlywheelVelocityTolerance) && // is this the intended 
                (getRPM(bottomFlywheelMotor) >= desiredRightRPM - ShooterConstants.FlywheelVelocityTolerance);// function?
    }

    public boolean isWithinIndexVelocityTolerance() {
        return getRPM(indexMotor) >= ShooterConstants.IndexRPM - ShooterConstants.IndexVelocityTolerance;
    }

    public boolean isNoteLoaded() {
        // return !breakBeam.get();
        return false;
    }

    public double getIndexRPM() {
        return getRPM(indexMotor) ;
    }

    /* LOGGING */

    private void logData() {
        Logger.recordOutput("Shooter/Top/MotorActualRPM", getRPM(topFlywheelMotor));
        Logger.recordOutput("Shooter/Top/FlywheelActualRPM", getRPM(topFlywheelMotor)
                * ShooterConstants.FlywheelGearRatio);
        Logger.recordOutput("Shooter/Top/ClosedLoopError", topFlywheelMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Shooter/Top/SupplyCurrent", topFlywheelMotor.getSupplyCurrent().getValue());

        Logger.recordOutput("Shooter/Bottom/MotorActualRPM", getRPM(bottomFlywheelMotor));
        Logger.recordOutput("Shooter/Bottom/FlywheelActualRPM", getRPM(bottomFlywheelMotor)
                * ShooterConstants.FlywheelGearRatio);
        Logger.recordOutput("Shooter/Bottom/ClosedLoopError", bottomFlywheelMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Shooter/Bottom/SupplyCurrent", bottomFlywheelMotor.getSupplyCurrent().getValue());

        Logger.recordOutput("Shooter/Index/ActualRPM", getRPM(indexMotor));
        Logger.recordOutput("Shooter/Index/ClosedLoopError", indexMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Shooter/Index/SupplyCurrent", indexMotor.getSupplyCurrent().getValue());
    }

    /* CONFIG */

    private void applyIndexMotorConfig(InvertedValue inversion) {
        TalonFXConfiguration PIDConfigs = new TalonFXConfiguration();
        PIDConfigs.Slot0.kP = ShooterConstants.IndexKP;
        PIDConfigs.Slot0.kI = ShooterConstants.IndexKI;
        PIDConfigs.Slot0.kD = ShooterConstants.IndexKD;
        PIDConfigs.Slot0.kV = ShooterConstants.IndexFF;
        PIDConfigs.Slot0.kS = ShooterConstants.IndexKS;

        applyMotorConfig(indexMotor, "indexMotor", PIDConfigs, inversion);
    }

    private void applyTopFlywheelMotorConfig(InvertedValue inversion) {
        TalonFXConfiguration PIDConfigs = new TalonFXConfiguration();
        PIDConfigs.Slot0.kP = ShooterConstants.FlywheelTopKP;
        PIDConfigs.Slot0.kI = ShooterConstants.FlywheelTopKI;
        PIDConfigs.Slot0.kD = ShooterConstants.FlywheelTopKD;
        PIDConfigs.Slot0.kV = ShooterConstants.FlywheelTopFF;

        applyMotorConfig(topFlywheelMotor, "TopFlywheelMotor", PIDConfigs, inversion);
    }

    private void applyBottomFlywheelMotorConfig(InvertedValue inversion) {
        TalonFXConfiguration PIDConfigs = new TalonFXConfiguration();
        PIDConfigs.Slot0.kP = ShooterConstants.FlywheelBottomKP;
        PIDConfigs.Slot0.kI = ShooterConstants.FlywheelBottomKI;
        PIDConfigs.Slot0.kD = ShooterConstants.FlywheelBottomKD;
        PIDConfigs.Slot0.kV = ShooterConstants.FlywheelBottomFF;

        applyMotorConfig(bottomFlywheelMotor, "BottomFlywheelMotor", PIDConfigs, inversion);
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
        PhysicsSim.getInstance().addTalonFX(topFlywheelMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(bottomFlywheelMotor, 0.001);
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