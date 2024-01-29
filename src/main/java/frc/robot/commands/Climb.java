// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj2.command.Command;
// // import frc.robot.subsystems.ShooterPosition;

// public class Climb extends Command {

//     private ShooterPosition elevatorSub;

//     public enum ClimbState {
//         StartingPosition,
//         WaitingForElevatorToReady,
//         MovingToHighBar,
//         LettingGoMidBar,
//         MovingToTraversalBar,
//         LettingGoHighBar,
//         GoingToRestingPosition,
//         Finished;

//         public static int valueOf(ClimbState e) {
//             int result = 0;
//             for (ClimbState v : values()) {
//                 if (v == e) break;
//                 else result++;
//             }
//             return result;
//         }
//     };

//     ClimbState currentState = ClimbState.StartingPosition;

//     public Climb(ShooterPosition e) {
//         elevatorSub = e;
//         addRequirements(elevatorSub);
//     }

//     @Override
//     public void initialize() {

//     }

//     @Override
//     public void execute() {
//     }

//     @Override
//     public void end(boolean interrupted) {
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
