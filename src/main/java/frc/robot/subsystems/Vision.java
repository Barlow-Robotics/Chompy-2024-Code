// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.kFallbackVisionStrategy;
import static frc.robot.Constants.VisionConstants.kMultiTagStdDevs;
import static frc.robot.Constants.VisionConstants.kPoseCameraName;
import static frc.robot.Constants.VisionConstants.kRobotToCam;
import static frc.robot.Constants.VisionConstants.kSingleTagStdDevs;
import static frc.robot.Constants.VisionConstants.kFieldTagLayout;
import static frc.robot.Constants.VisionConstants.kTargetCameraName;
import static frc.robot.Constants.VisionConstants.kPrimaryVisionStrategy;

//import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    private PhotonCamera targetCamera;
    private PhotonCamera poseCamera;
    private /* final */ PhotonPoseEstimator photonEstimator;
    private double lastEstTimestamp = 0;

    private PhotonCameraSim poseCameraSim;
    private VisionSystemSim visionSim;
    private Transform3d robotToCamera;

    boolean aprilTagDetected = false;

    public Vision() /* throws IOException */ {
        targetCamera = new PhotonCamera(kTargetCameraName);
        poseCamera = new PhotonCamera(kPoseCameraName);

        photonEstimator = new PhotonPoseEstimator(kFieldTagLayout, kPrimaryVisionStrategy, poseCamera, kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(kFallbackVisionStrategy);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the
            // field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this
            // simulated field.
            visionSim.addAprilTags(kFieldTagLayout);
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
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(poseCameraSim, kRobotToCam);

            poseCameraSim.enableDrawWireframe(true);
        }

        // catch (IOException e) {
        // // TODO decide what you want to do if the layout fails to load
        // photonEstimator = new PhotonPoseEstimator(null, null, camera, null);
        // }
        // Pose3d robotPose =
        // PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
        // fieldTags.getTagPose(target.getFiducialId()), robotToCamera);
    }

    public void initSendable(SendableBuilder builder) {
        // builder.addDoubleProperty("Estimated Global Pose",
        // this::getEstimatedGlobalPose, null);
    }
    
    public PhotonPipelineResult getLatestPoseResult() {
         return poseCamera.getLatestResult();
    } 

    public void periodic() {
        var result = getLatestPoseResult();

        if ( result.hasTargets()) {
            var target = result.getBestTarget() ;   
            
            var toTarget = target.getBestCameraToTarget() ;
            var tagPose = kFieldTagLayout.getTagPose(target.getFiducialId()).orElse(new Pose3d()); 
            var transform3d = new Transform3d();

            Pose3d robotPose = 
                PhotonUtils.estimateFieldToRobotAprilTag(
                    toTarget, tagPose, 
                    transform3d);

            double range =
                PhotonUtils.calculateDistanceToTargetMeters(
                        1,
                        .1,
                        Units.degreesToRadians( 20.0),
                        Units.degreesToRadians(result.getBestTarget().getPitch()));

            NetworkTableInstance.getDefault().getEntry("distanceFromX")
                        .setDouble(robotPose.getX());
            NetworkTableInstance.getDefault().getEntry("distanceFromY")
                            .setDouble(robotPose.getY());
            NetworkTableInstance.getDefault().getEntry("distanceFromZ")
                            .setDouble(robotPose.getZ());
        }

        var poseEstimate = getEstimatedGlobalPose() ;
        if ( !poseEstimate.isEmpty()) {
            int wpk = 1 ;
        }
    }
            // SmartDashboard.putData(getEstimatedGlobalPose());
        
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
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = getLatestPoseResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;
        // Decrease stnd devs if multiple targets are visible
        if (numTags > 1)
            estStdDevs = kMultiTagStdDevs;
        // Increase stnd devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

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

    public double getAprilTagDistToCenter() {
        return 0;
    }

    public boolean getAprilTagDetected() {
        return getLatestPoseResult().hasTargets();
    }

    // private void addNetworkTableEntries() {
    //     NetworkTableInstance.getDefault().getEntry("vision/xPosition").setDouble(0.0); // CHANGE
    //     NetworkTableInstance.getDefault().getEntry("vision/yPosition").setDouble(0.0); // CHANGE
    //     NetworkTableInstance.getDefault().getEntry("vision/zPosition").setDouble(0.0); // CHANGE
    // }
}
