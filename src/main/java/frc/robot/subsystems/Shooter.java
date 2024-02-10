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
    private final TalonFX lowerFlywheelMotor;
    private final TalonFXSimState lowerFlywheelMotorSim;
    private final DCMotorSim lowerMotorModel = 
        new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1), 1, ShooterConstants.jKgMetersSquared);
    
    private final TalonFX upperFlywheelMotor;
    private final TalonFXSimState upperFlywheelMotorSim;
    private final DCMotorSim upperMotorModel = 
        new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1), 1, ShooterConstants.jKgMetersSquared);

    private final TalonFX indexMotor;
    private final TalonFXSimState indexMotorSim;
    private final DCMotorSim indexMotorModel = 
        new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1), 1, ShooterConstants.jKgMetersSquared);

    
    private final VelocityVoltage voltageVelocity = 
        new VelocityVoltage(0, 0, true, 0, 0, 
                            false, false, false);
    private final NeutralOut brake = new NeutralOut();

    DigitalInput breakBeam;
    DIOSim breakBeamSim;

    private boolean simulationInitialized = false;

    // private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    // private GenericEntry shuffleBoardSpeed = tab.add("ShuffleBoard Speed", 1).getEntry();

    public Shooter() {
        lowerFlywheelMotor = new TalonFX(ElectronicsIDs.LowerShooterMotorID); 
        upperFlywheelMotor = new TalonFX(ElectronicsIDs.UpperShooterMotorID); 
        indexMotor = new TalonFX(ElectronicsIDs.IndexMotorID); 
        
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        setTalonConfigs(talonConfigs);
        applyMotorConfigs(
            lowerFlywheelMotor, "lowerShooterMotor", 
            talonConfigs, motorOutputConfigs, InvertedValue.CounterClockwise_Positive);
        applyMotorConfigs(
            upperFlywheelMotor, "upperShooterMotor", 
            talonConfigs, motorOutputConfigs, InvertedValue.Clockwise_Positive);

        lowerFlywheelMotorSim = lowerFlywheelMotor.getSimState();
        upperFlywheelMotorSim = upperFlywheelMotor.getSimState();
        indexMotorSim = indexMotor.getSimState();

        breakBeam = new DigitalInput(ElectronicsIDs.BreakBeamID);
        
    }

    @Override
    public void periodic() {
        logData();
    }

    public void startFlywheels(double shooterRPM) {
        // if (shuffleBoardSpeed.getDouble(-1.0) <= Constants.Falcon500MaxRPM / 60) { // max rps: 105
        //         shooterRPM = shuffleBoardSpeed.getDouble(-1.0);
        // }
        Logger.recordOutput("Shooter/DesiredShooterRPM", shooterRPM);

        double shooterRPS = shooterRPM / 60;

        lowerFlywheelMotor.setControl(voltageVelocity.withVelocity(shooterRPS));
        upperFlywheelMotor.setControl(voltageVelocity.withVelocity(shooterRPS));

        Logger.recordOutput("Shooter/WithinToleranceFlywheels", isWithinVelocityTolerance(shooterRPS));
    }

    public void startIndex(double indexRPM) {
        Logger.recordOutput("Shooter/DesiredIndexRPM", indexRPM);

        indexMotor.setControl(voltageVelocity.withVelocity(indexRPM));
    }

    public void stop() {
        lowerFlywheelMotor.setControl(brake);
        upperFlywheelMotor.setControl(brake);
        indexMotor.setControl(brake);
    }

    private double getRPM(TalonFX motor) {
        return motor.getVelocity().getValue()*Constants.SecondsPerMinute;
    }

    private boolean isWithinVelocityTolerance(double desiredRPM) {
        return (getRPM(lowerFlywheelMotor) >= Math.abs(Constants.LowerToleranceLimit * desiredRPM)) &&
                (getRPM(lowerFlywheelMotor) <= Math.abs(Constants.UpperToleranceLimit * desiredRPM)) &&
                (getRPM(upperFlywheelMotor) >= Math.abs(Constants.LowerToleranceLimit * desiredRPM)) &&
                (getRPM(upperFlywheelMotor) <= Math.abs(Constants.UpperToleranceLimit * desiredRPM));
    }

    public boolean isNoteLoaded() {
        // return breakBeam.get();
        return false;
    }

    /* LOGGING */

    private void logData() {
        Logger.recordOutput("Shooter/ActualRPMLower", getRPM(lowerFlywheelMotor));
        Logger.recordOutput("Shooter/ActualRPMUpper", getRPM(upperFlywheelMotor));
        Logger.recordOutput("Shooter/ActualRPMIndex", getRPM(indexMotor));
        Logger.recordOutput("Shooter/Is/Indexing", isWithinVelocityTolerance(ShooterConstants.IndexRPM));
        Logger.recordOutput("Shooter/ClosedLoopErrorLower", lowerFlywheelMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Shooter/ClosedLoopErrorUpper", upperFlywheelMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Shooter/Is/NoteLoaded", isNoteLoaded());
        Logger.recordOutput("Shooter/Is/ShootingAmp", isWithinVelocityTolerance(ShooterConstants.AmpRPM));
        Logger.recordOutput("Shooter/Is/ShootingSpeaker", isWithinVelocityTolerance(ShooterConstants.SpeakerRPM));
        Logger.recordOutput("Shooter/Is/ShootingTrap", isWithinVelocityTolerance(ShooterConstants.TrapRPM));
        Logger.recordOutput("Shooter/Is/IntakingSource", isWithinVelocityTolerance(ShooterConstants.SourceRPM));
        Logger.recordOutput("Shooter/Is/IntakingFloor", isWithinVelocityTolerance(ShooterConstants.FloorRPM));
        Logger.recordOutput("Shooter/CurrentSupply/FlywheelLower", lowerFlywheelMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Shooter/CurrentSupply/FlywheelUpper", upperFlywheelMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Shooter/CurrentSupply/Index", indexMotor.getSupplyCurrent().getValue());
    }

    /* CONFIG */

    private void setTalonConfigs(TalonFXConfiguration configs) {
        configs.Slot0.kP = ShooterConstants.ShooterKP;
        configs.Slot0.kI = ShooterConstants.ShooterKI;
        configs.Slot0.kD = ShooterConstants.ShooterKD;
        configs.Slot0.kV = ShooterConstants.ShooterFF;
    }

    private void applyMotorConfigs(TalonFX motor, String motorName, 
        TalonFXConfiguration talonConfigs, MotorOutputConfigs motorOutputConfigs, InvertedValue inversion) {
        
        motorOutputConfigs.Inverted = inversion;

        // set current limit
        CurrentLimitsConfigs currentLimitConfigs = talonConfigs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        
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
            System.out.println("Could not apply motor output configs to " + motorName + " error code: " + status.toString());
        }
    }
    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(lowerFlywheelMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(upperFlywheelMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(indexMotor, 0.001);

        lowerFlywheelMotorSim.Orientation = ChassisReference.CounterClockwise_Positive; // CHANGE
        upperFlywheelMotorSim.Orientation = ChassisReference.Clockwise_Positive; // CHANGE

        breakBeamSim = new DIOSim(breakBeam);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        lowerFlywheelMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double lowerVoltage = lowerFlywheelMotorSim.getMotorVoltage();
        lowerMotorModel.setInputVoltage(lowerVoltage);
        lowerMotorModel.update(0.02);
        lowerFlywheelMotorSim.setRotorVelocity(lowerMotorModel.getAngularVelocityRPM() / 60.0);
        lowerFlywheelMotorSim.setRawRotorPosition(lowerMotorModel.getAngularPositionRotations());

        upperFlywheelMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double upperVoltage = upperFlywheelMotorSim.getMotorVoltage();
        upperMotorModel.setInputVoltage(upperVoltage);
        upperMotorModel.update(0.02);
        upperFlywheelMotorSim.setRotorVelocity(upperMotorModel.getAngularVelocityRPM() / 60.0);
        upperFlywheelMotorSim.setRawRotorPosition(upperMotorModel.getAngularPositionRotations());

        indexMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double indexVoltage = indexMotorSim.getMotorVoltage();
        indexMotorModel.setInputVoltage(indexVoltage);
        indexMotorModel.update(0.02);
        indexMotorSim.setRotorVelocity(indexMotorModel.getAngularVelocityRPM() / 60.0);
        indexMotorSim.setRawRotorPosition(indexMotorModel.getAngularPositionRotations());
    }

}