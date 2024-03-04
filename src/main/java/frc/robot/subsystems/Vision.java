
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.FallbackVisionStrategy;
import static frc.robot.Constants.VisionConstants.MultiTagStdDevs;
import static frc.robot.Constants.VisionConstants.PoseCameraName;
import static frc.robot.Constants.VisionConstants.PoseCameraToRobot;
import static frc.robot.Constants.VisionConstants.RobotToTargetCam;
import static frc.robot.Constants.VisionConstants.SingleTagStdDevs;
import static frc.robot.Constants.VisionConstants.FieldTagLayout;
import static frc.robot.Constants.VisionConstants.TargetCameraName;
import static frc.robot.Constants.VisionConstants.PrimaryVisionStrategy;

import java.util.Hashtable;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    private PhotonCamera targetCamera;
    private PhotonCamera poseCamera;
    public final PhotonPoseEstimator photonEstimator;
    private double lastEstTimestamp = 0;

    private PhotonCameraSim poseCameraSim;
    private PhotonCameraSim targetCameraSim;
    private VisionSystemSim visionSim;
    private Transform3d robotToCamera;
    private PhotonTrackedTarget currentBestTarget;
    private PhotonTrackedTarget currentBestAlignTarget;
    public List<PhotonTrackedTarget> allDetectedTargets;
    private HashSet<Integer> targetAlignSet;
    public OptionalInt activeAlignTargetId;
    private Alliance alliance;


    boolean aprilTagDetected = false;

    public enum TargetToAlign {
        Speaker, Amp, Source, Stage, /* Note */
    }


    Hashtable<Integer, Integer> blueTrackableIDs = new Hashtable<>() ;
    Hashtable<Integer, Integer> redTrackableIDs = new Hashtable<>() ;

    public Vision() /* throws IOException */ {
        targetCamera = new PhotonCamera(TargetCameraName);
        poseCamera = new PhotonCamera(PoseCameraName);

        photonEstimator = new PhotonPoseEstimator(FieldTagLayout, PrimaryVisionStrategy, poseCamera, PoseCameraToRobot);
        photonEstimator.setMultiTagFallbackStrategy(FallbackVisionStrategy);

        alliance = DriverStation.Alliance.Red;
        if (DriverStation.isEnabled()) {
            var a = DriverStation.getAlliance();
            if (a.isPresent()) {
                alliance = a.get();
            }
        }

        targetAlignSet = new HashSet<Integer>();
        activeAlignTargetId = OptionalInt.empty();


        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the
            // field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this
            // simulated field.
            visionSim.addAprilTags(FieldTagLayout);
            // Create simulated camera properties. These can be set to mimic your actual
            // camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values
            // with visible
            // targets.
            poseCameraSim = new PhotonCameraSim(poseCamera, cameraProp);
            targetCameraSim = new PhotonCameraSim(targetCamera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.

            visionSim.addCamera(poseCameraSim, PoseCameraToRobot);
            visionSim.addCamera(targetCameraSim, RobotToTargetCam);

            poseCameraSim.enableDrawWireframe(true);
        }

        // catch (IOException e) {
        // // TODO decide what you want to do if the layout fails to load
        // photonEstimator = new PhotonPoseEstimator(null, null, camera, null);
        // }
        // Pose3d robotPose =
        // PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
        // fieldTags.getTagPose(target.getFiducialId()), robotToCamera);

        blueTrackableIDs.put( 7, 7) ; // speaker
        blueTrackableIDs.put( 6, 6) ; // amp
        blueTrackableIDs.put( 1, 1) ; // source
        blueTrackableIDs.put( 2, 2) ; // source

        redTrackableIDs.put( 4, 4) ; // speaker
        redTrackableIDs.put( 5, 5) ; // amp
        redTrackableIDs.put( 10, 10) ; // source
        redTrackableIDs.put( 9, 9) ; // source

    }



    public void alignTo(TargetToAlign target) {
        targetAlignSet.clear();
        // aligningWithNote = false;
        // if (target == TargetToAlign.Note) { // Not sure that this works
        // aligningWithNote = true;
        // } else
        if (alliance == DriverStation.Alliance.Blue) {
            switch (target) {
                case Speaker:
                    targetAlignSet.add(7);
                    targetAlignSet.add(8);
                    break;
                case Source:
                    targetAlignSet.add(2);
                    targetAlignSet.add(1);
                    break;
                case Amp:
                    targetAlignSet.add(6);
                    break;
                case Stage:
                    targetAlignSet.add(15);
                    targetAlignSet.add(14);
                    targetAlignSet.add(16);
                    break;

            }
        } else {
            switch (target) {
                case Speaker:
                    targetAlignSet.add(3);
                    targetAlignSet.add(4);
                    break;
                case Source:
                    targetAlignSet.add(9);
                    targetAlignSet.add(10);
                    break;
                case Amp:
                    targetAlignSet.add(5);
                    break;
                case Stage:
                    targetAlignSet.add(11);
                    targetAlignSet.add(12);
                    targetAlignSet.add(13);
                    break;
            }
        }
    }




    public void periodic() {

        // TODO: feed this pose estimate back to the combined pose estimator in drive
        // var poseEstimate = getEstimatedGlobalPose();

        // Find all the results from the tracking camera
        var tracking_result = getLatestTrackingResult();

        // Update the current bestAlignTarget based on the chosen target
        currentBestAlignTarget = null;

        if (tracking_result.isPresent()) {

            if (tracking_result.get().hasTargets()) {

                allDetectedTargets = tracking_result.get().getTargets();
                currentBestTarget = tracking_result.get().getBestTarget();
            }
            if (activeAlignTargetId.isPresent()) {
                for (PhotonTrackedTarget target : allDetectedTargets) {
                    if (target.getFiducialId() == activeAlignTargetId.getAsInt()) {
                        currentBestAlignTarget = target;
                        break;
                    }
                }
            }

        }

        advantageKitLogging();
    }





    ///////////////////////////////////
    //   Pose stuff
    ///////////////////////////////////


    public PhotonPipelineResult getLatestPoseResult() {

            return poseCamera.getLatestResult();
    }

    public Optional<PhotonPipelineResult> getLatestTrackingResult() {
        if (targetCamera.isConnected()) {
            return Optional.of(targetCamera.getLatestResult());
        } else {
            return Optional.empty();
        }
    }




    ///////////////////////////////////
    //   Tracking stuff
    ///////////////////////////////////

    // return the first target in the list with an ID we are interested in given our alliance color
    // This uses the simple approach and assumes that we won't be seeing multiple that we're interested in or that
    // the first one in the list is the best one.
    public Optional<PhotonTrackedTarget> getBestTrackableTarget() {
        if (DriverStation.getAlliance().isPresent()) {
            for (var tempTarget : allDetectedTargets) {
                if (DriverStation.getAlliance().get() == Alliance.Red && redTrackableIDs.contains(tempTarget.getFiducialId())) {
                    return Optional.of(tempTarget) ;
                }
                if (DriverStation.getAlliance().get() == Alliance.Blue && blueTrackableIDs.contains(tempTarget.getFiducialId())) {
                    return Optional.of(tempTarget) ;
                }
            }
        }
        return Optional.empty() ;
    }


    public Optional<PhotonTrackedTarget> getTarget(int id) {
        for (var tempTarget : allDetectedTargets) {
            if ((tempTarget.getFiducialId() == id) ) {
                return Optional.of(tempTarget) ;
            }
        }
        return Optional.empty() ;
    }


    public Optional<PhotonTrackedTarget> getSpeakerTarget() {
    if (DriverStation.getAlliance().isPresent()) {
        for (var tempTarget : allDetectedTargets) {
            if (DriverStation.getAlliance().get() == Alliance.Red && tempTarget.getFiducialId() == VisionConstants.RedSpeakerCenterAprilTagID) {
                return Optional.of(tempTarget) ;
            }
            if (DriverStation.getAlliance().get() == Alliance.Blue && tempTarget.getFiducialId() == VisionConstants.BlueSpeakerCenterAprilTagID) {
                return Optional.of(tempTarget) ;
            }
        }
    }
    return Optional.empty() ;
}






    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets
     *         used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        if (poseCamera.isConnected()) {
            var visionEst = photonEstimator.update();
            double latestTimestamp = poseCamera.getLatestResult().getTimestampSeconds();
            boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est -> getSimDebugField()
                                .getObject("VisionEstimation")
                                .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            if (newResult)
                                getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }
            if (newResult)
                lastEstTimestamp = latestTimestamp;
            return visionEst;
        } else {
            return Optional.empty();
        }
    }


    // // wpk not used anywhere
    // public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    //     var estStdDevs = SingleTagStdDevs;
    //     var targets = getLatestPoseResult().getTargets();
    //     int numTags = 0;
    //     double avgDist = 0;
    //     for (var tgt : targets) {
    //         var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
    //         if (tagPose.isEmpty())
    //             continue;
    //         numTags++;
    //         avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    //     }
    //     if (numTags == 0)
    //         return estStdDevs;
    //     avgDist /= numTags;
    //     // Decrease stnd devs if multiple targets are visible
    //     if (numTags > 1)
    //         estStdDevs = MultiTagStdDevs;
    //     // Increase stnd devs based on (average) distance
    //     if (numTags == 1 && avgDist > 4)
    //         estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    //     else
    //         estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    //     return estStdDevs;
    // }

    // ----- Simulation

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation())
            visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation())
            return null;
        return visionSim.getDebugField();
    }



    // public void chooseBestTarget() {
    //     activeAlignTargetId = OptionalInt.empty();
    //     if (allDetectedTargets != null) {
    //         for (PhotonTrackedTarget target : allDetectedTargets) {
    //             if (targetAlignSet.contains(target.getFiducialId())) {
    //                 activeAlignTargetId = OptionalInt.of(target.getFiducialId());
    //                 return;
    //             }
    //         }
    //     }
    // }


    // public OptionalDouble getTargetRotOffSet() {
    //     if (currentBestAlignTarget != null) {
    //         return OptionalDouble.of(currentBestAlignTarget.getYaw());
    //     }
    //     return OptionalDouble.empty();
    // }

    // public OptionalDouble getTargetLateralOffSet() {
    //     if (currentBestAlignTarget != null) {
    //         // The pose of the april tag in the camera reference frame
    //         var targetPose = currentBestAlignTarget.getBestCameraToTarget();

    //         // targetYaw is the offset angle from camera-forward to the target
    //         var targetYaw = currentBestAlignTarget.getYaw();

    //         // Now find the "pointing angle of the target" in the reference frame
    //         // of the robot. If you drew a ray from the target in space this would be
    //         // the "center line" the robot wants to reach laterally.

    //         // Note: this rotation value is a bit counter-intuitive. A Z-axis rotation of 0
    //         // actually means the April tag is pointed in the same direction as the camera
    //         var targetPointing = targetPose.getRotation().getZ();
    //         // We want an angle about 0 when the tag is facing us so shift by Pi
    //         // and then unroll.
    //         targetPointing += Math.PI;
    //         if (targetPointing > Math.PI) {
    //             targetPointing -= 2 * Math.PI;
    //         } else if (targetPointing < -Math.PI) {
    //             targetPointing += 2 * Math.PI;
    //         }
    //         // Now combine the 2 angles to get the total angle between the robot and
    //         // this center-line.
    //         var combinedAngle = Math.toRadians(targetYaw) - targetPointing;

    //         var targetDist = targetPose.getTranslation().getNorm();

    //         // Finally, the lateral distance can be found by trig:
    //         // Sin(angle) = Lateral / Hypotenuse
    //         // Lateral = Hypotenuse * Sin(combined angle)
    //         var lateralDist = Math.sin(combinedAngle) * targetDist;

    //         return OptionalDouble.of(lateralDist);
    //     }
    //     return OptionalDouble.empty();
    // }

    // public OptionalDouble getTargetDistance() {
    //     if (currentBestAlignTarget != null) {
    //         var targetPose = currentBestAlignTarget.getBestCameraToTarget();
    //         var targetDist = targetPose.getTranslation().getNorm();

    //         return OptionalDouble.of(targetDist);
    //     }
    //     return OptionalDouble.empty();
    // }

    private void advantageKitLogging() {
        if (robotToCamera != null) {
            Logger.recordOutput("vision/xPosition", robotToCamera.getX());
            Logger.recordOutput("vision/yPosition", robotToCamera.getY());
            Logger.recordOutput("vision/zPosition", robotToCamera.getZ());
        }

        if (currentBestTarget != null) {
            Logger.recordOutput("vision/currentBestFiducial", currentBestTarget.getFiducialId());
            Logger.recordOutput("vision/bestCameraToTarget", currentBestTarget.getBestCameraToTarget());
        }
        // if (currentBestAlignTarget != null) {
        //     Logger.recordOutput("vision/target/RotOffset", getTargetRotOffSet().getAsDouble());
        //     Logger.recordOutput("vision/target/LateralOffset", getTargetLateralOffSet().getAsDouble());
        //     Logger.recordOutput("vision/target/Distance", getTargetDistance().getAsDouble());
        // }

        Logger.recordOutput("vision/targetAlignSet", targetAlignSet.toString());
        Logger.recordOutput("vision/activeAlignTargetStr", activeAlignTargetId.toString());
        if (activeAlignTargetId.isPresent()) {
            Logger.recordOutput("vision/activeAlignTarget", activeAlignTargetId.getAsInt());
        }
    }

    // private void addNetworkTableEntries() {
    // NetworkTableInstance.getDefault().getEntry("vision/xPosition").setDouble(0.0);
    // // CHANGE
    // NetworkTableInstance.getDefault().getEntry("vision/yPosition").setDouble(0.0);
    // // CHANGE
    // NetworkTableInstance.getDefault().getEntry("vision/zPosition").setDouble(0.0);
    // // CHANGE
    // }

    // public double getSpeakerAprilTagPitch() {
    //     // returns InvalidAngle constant if not facing speaker

    //     var poseResult = getLatestPoseResult();
    //     if (!poseResult.hasTargets())
    //         return (VisionConstants.InvalidAngle);

    //     var target = poseResult.getBestTarget();
    //     int targetAprilTagID = target.getFiducialId();

    //     // check that target is the Speaker's center AprilTag
    //     if (targetAprilTagID == VisionConstants.NullAprilTagID) {
    //         // Vision has no target acquired
    //         return (VisionConstants.InvalidAngle);
    //     }
    //     // If the primary target is not a speaker center AprilTag...
    //     else if (targetAprilTagID != VisionConstants.RedSpeakerCenterAprilTagID
    //             && targetAprilTagID != VisionConstants.BlueSpeakerCenterAprilTagID) {
    //         // Walk the list of found AprilTags aside from the primary
    //         // to see if desired speaker center AprilTag is in the list
    //         List<PhotonTrackedTarget> targets = poseResult.getTargets();
    //         target = null;
    //         for (var tempTarget : targets) {
    //             if ((tempTarget.getFiducialId() == VisionConstants.RedSpeakerCenterAprilTagID) ||
    //                     tempTarget.getFiducialId() == VisionConstants.BlueSpeakerCenterAprilTagID)
    //             {
    //                 target = tempTarget;
    //                 break;
    //             }
    //         }
    //     }
    //     if (target != null){
    //         return target.getPitch();
    //     }
    //     else {
    //         return(VisionConstants.InvalidAngle);
    //     }
    // }

    // public double getSpeakerTargetDistance()
    // {
    //     var poseResult = getLatestPoseResult();
    //     if (!poseResult.hasTargets())
    //         return (VisionConstants.NoTargetDistance);
    
    //     var target = poseResult.getBestTarget();
    //     int targetAprilTagID = target.getFiducialId();
    
    //     // check that target is the Speaker's center AprilTag
    //     // Does this just duplicate the poseResults.hasTargets() test above???
    //     if (targetAprilTagID == VisionConstants.NullAprilTagID) {
    //         // Vision has no target acquired
    //         return (VisionConstants.NoTargetDistance);
    //     }

    //     // If the primary target is not a speaker center AprilTag...
    //     target = null;
    //     if (targetAprilTagID != VisionConstants.RedSpeakerCenterAprilTagID
    //             && targetAprilTagID != VisionConstants.BlueSpeakerCenterAprilTagID) 
    //     {
    //         // Walk the list of found AprilTags aside from the primary
    //         // to see if desired speaker center AprilTag is in the list
    //         List<PhotonTrackedTarget> targets = poseResult.getTargets();
    //         for (var tempTarget : targets) {
    //             if ((tempTarget.getFiducialId() == VisionConstants.RedSpeakerCenterAprilTagID) ||
    //                     tempTarget.getFiducialId() == VisionConstants.BlueSpeakerCenterAprilTagID)
    //             {
    //                 target = tempTarget;
    //                 break;
    //             }
    //         }
    //     }
    
    //     if (target != null)
    //         // Is the X value is the distance to the AprilTag?
    //         // Or do I need to compute it as the length of the hypotenuse
    //         //	where X and Y are the sides of the right angle
    //         // Or, maybe we can use getTargetDistance() to find this?
    //         //	==> This method doesn't find the Speaker AprilTag we need.
    //         //	==>  It just uses the target that is currently "best aligned"
    //         return(target.getBestCameraToTarget().getX());
    //     else
    //         // What should we return when there is no target?
    //         return(VisionConstants.NoTargetDistance);
    // }




    // public boolean noteIsVisible() {
    // return this.noteDetected;
    // }

    // public boolean isAligningWithNote() {
    // return aligningWithNote;
    // }

    // public double getNoteDistanceFromCenter() {
    // // tell how many pixels the note is from the center of the screen.
    // return this.noteDistanceFromCenter;
    // }

    // public double getNoteHeight() {
    // return this.noteHeight;
    // }

    // public double getNoteWidth() {
    // return this.noteWidth;
    // }
}
