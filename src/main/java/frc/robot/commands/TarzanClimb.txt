// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//No copyright? Nothing was in here when I opened it

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmBar;
//import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmBarConstants;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;

public class Climb extends CommandBase {

    private ArmBar m_armBar;

    public enum ArmCommandState {
        ResettingPosition,
        //FreeingBClaws ,
        WaitingForArmToReady,
        MovingToHighBar,
        LettingGoMidBar,
        MovingToTraversalBar,
        LettingGoHighBar,
        GoingToRestingPosition ,
        //OnTraversalBar,
        Finished ;

        public static int valueOf(ArmCommandState e) {
            int result = 0 ;
            for (ArmCommandState v : values()) {
                if ( v == e ) {
                    break ;
                } else {
                    result++ ;
                }
            }
            return result;
        }
    };

    ArmCommandState currentState = ArmCommandState.ResettingPosition;
    Joystick controller ;
    Joystick gamePad ;
//    NavigationSubsystem nav ;


    /** Creates a new Climb. */
    public Climb(ArmBar a, Joystick c, Joystick gp) { 

        // Use addRequirements() here to declare subsystem dependencies.
        m_armBar = a;
       // nav = n ;
        addRequirements(m_armBar);
        // m_drive = d;
        controller = c ;
        gamePad = gp ;

        report() ;        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //currentState = ArmCommandState.FreeingBClaws;
        currentState = ArmCommandState.ResettingPosition;

    }

    double latchedAngle ;

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // NetworkTableInstance inst = NetworkTableInstance.getDefault() ;
        // inst.getEntry("climb_command/climb_button").setBoolean(gamePad.getRawButton(Constants.Fight_Stick.Button_Y)) ;

        switch (currentState) {
            case ResettingPosition:
                m_armBar.setHighSpeed();
                m_armBar.resetPosition(0.0);
                // currentState = ArmCommandState.FreeingBClaws;
                currentState = ArmCommandState.WaitingForArmToReady;
                break ;

            // case FreeingBClaws:
            //     if (m_armBar.getArmAngle() > -(ArmBarConstants.FreeingBClawAngle - ArmBarConstants.AngleTolerance) ) {
            //         m_armBar.rotateGripperArmDegree(-ArmBarConstants.FreeingBClawAngle);
            //     } else {
            //         currentState = ArmCommandState.WaitingForArmToReady;
            //     }
            //     break;

            case WaitingForArmToReady:
                if (Math.abs(m_armBar.getArmAngle() - ArmBarConstants.MidBarRotationAngle) > ArmBarConstants.AngleTolerance) {
                    m_armBar.rotateGripperArmDegree(ArmBarConstants.MidBarRotationAngle);
                } else {
                    if (gamePad.getRawButton(Constants.Logitech_F310_Controller.Button_X)) {
                        // System.out.format("Climb Button pressed in %s. Arm angle is %5.2f, pitch is %5.2f", currentState.name(), m_armBar.getArmAngle(), nav.getPitch()) ;
                        m_armBar.setHighSpeed();
                        latchedAngle = m_armBar.getArmAngle() ;
                        currentState = ArmCommandState.MovingToHighBar;
                    }
                }
                break;


            case MovingToHighBar:
                // if the arm is not above the high bar, move the arm higher.
                // note: this attempts to take the pitch of the chassis into accoun, so the arm may need to rotate more if the
                // bot is pitched forward.
//                if (Math.abs ((m_armBar.getArmAngle()-nav.getPitch()) - ArmBarConstants.HighBarRotationAngle) > ArmBarConstants.AngleTolerance) {
                if (Math.abs ((m_armBar.getArmAngle()) - (ArmBarConstants.HighBarRotationAngle)) > 2.0) {
                    // this is imprecise. It just adds a constant to the minimum angle in case it need to keep rotating
                    m_armBar.rotateGripperArmDegree(ArmBarConstants.HighBarRotationAngle);
//                    m_armBar.rotateGripperArmDegree(ArmBarConstants.HighBarRotationAngle);
                } else {
                    // if (gamePad.getRawButton(Constants.Fight_Stick.Button_Y)) {
                        m_armBar.setSlowSpeed();
                        latchedAngle = m_armBar.getArmAngle() ;
                        currentState = ArmCommandState.LettingGoMidBar;
                    // }
                }
                break ;

            case LettingGoMidBar:
                // can we do something with pitch here?
                if (Math.abs ((m_armBar.getArmAngle()) - (latchedAngle-ArmBarConstants.ReleaseMidBarAngle)) > 2.0) {
                    // this is imprecise. It just adds a constant to the minimum angle in case it need to keep rotating
                    m_armBar.rotateGripperArmDegree(latchedAngle-ArmBarConstants.ReleaseMidBarAngle);
                } else {
                    // if (gamePad.getRawButton(Constants.Fight_Stick.Button_Y)) {
                        m_armBar.setHighSpeed();
                        //System.out.format("Climb Button pressed in %s. Arm angle is %5.2f, pitch is %5.2f", currentState.name(), m_armBar.getArmAngle(), nav.getPitch()) ;
                        latchedAngle = m_armBar.getArmAngle() ;
                    currentState = ArmCommandState.MovingToTraversalBar;
                    // }
                }
                break;

            case MovingToTraversalBar:
                // if the arm is not above the traverse bar, move the arm higher.
                // note: this attempts to take the pitch of the chassis into accoun, so the arm may need to rotate more if the
                // bot is pitched forward.
                if (Math.abs ((m_armBar.getArmAngle()) - (ArmBarConstants.TraverseBarRotationAngle)) > 2.0) {
                    // this is imprecise. It just adds a constant to the minimum angle in case it need to keep rotating
                    m_armBar.rotateGripperArmDegree(ArmBarConstants.TraverseBarRotationAngle);
                } else {
                    // if (gamePad.getRawButton(Constants.Fight_Stick.Button_Y)) {
                        m_armBar.setSlowSpeed();
                        latchedAngle = m_armBar.getArmAngle() ;
                        currentState = ArmCommandState.LettingGoHighBar;
                    // }
                }
                break ;

            case LettingGoHighBar:
                // can we do something with pitch here?
                if (Math.abs ((m_armBar.getArmAngle()) - (latchedAngle-ArmBarConstants.ReleaseHighBarAngle)) > 2.0) {
                    // this is imprecise. It just adds a constant to the minimum angle in case it need to keep rotating
                    m_armBar.rotateGripperArmDegree((latchedAngle-ArmBarConstants.ReleaseHighBarAngle));
                } else {
                    // if (gamePad.getRawButton(Constants.Fight_Stick.Button_Y)) {
                        latchedAngle = m_armBar.getArmAngle() ;
                        currentState = ArmCommandState.GoingToRestingPosition;
                    // }
                }
                break;
            case GoingToRestingPosition:
            if (Math.abs ((m_armBar.getArmAngle()) - (latchedAngle+90)) > 2.0) {
                // this is imprecise. It just adds a constant to the minimum angle in case it need to keep rotating
                m_armBar.rotateGripperArmDegree((latchedAngle+90));
            } else {
                // if (gamePad.getRawButton(Constants.Fight_Stick.Button_Y)) {
                    //System.out.format("Climb Button pressed in %s. Arm angle is %5.2f, pitch is %5.2f", currentState.name(), m_armBar.getArmAngle(), nav.getPitch()) ;
                    latchedAngle = m_armBar.getArmAngle() ;
                    currentState = ArmCommandState.Finished;
                // }
            }


            case Finished:
                // Bot is immobile - nothing else needs to go here
                m_armBar.stopMotor();
                break;
        }

        report();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_armBar.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return currentState == ArmCommandState.Finished ;
    }





    void report() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault() ;

        inst.getEntry("climb/climb_state").setString(currentState.name());
        inst.getEntry("climb/climb_state_num").setDouble(ArmCommandState.valueOf(currentState));
        inst.getEntry("climb/arm_angle").setDouble(m_armBar.getArmAngle()) ;
//        inst.getEntry("climb/pitch").setDouble(nav.getPitch()) ;
        // inst.getEntry("climb/pitch_plus_arm").setDouble(nav.getPitch()+m_armBar.getArmAngle()) ;
        // inst.getEntry("climb/roll").setDouble(nav.getRoll()) ;


    }

}