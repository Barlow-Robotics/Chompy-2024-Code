// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private double lastEstTimestamp = 0;

  
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;


  /** Creates a new Vision. */
  public Vision() {
    camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    camera = new PhotonCamera("Global_Shutter_Camera");
    
    photonEstimator =
                new PhotonPoseEstimator(
                        kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
