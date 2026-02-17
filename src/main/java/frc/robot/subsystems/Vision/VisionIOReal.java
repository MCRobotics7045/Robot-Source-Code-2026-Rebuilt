// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class VisionIOReal implements VisionIO {

    private PhotonCamera camera;
 

    private final Transform3d robotToCamera;
    

    public VisionIOReal(String camName,Transform3d robotToCamera) {
        this.camera = new PhotonCamera(camName);

        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(VisionIOinputs inputs) {
        inputs.CameraConnection = (camera.isConnected());
        Set<Short> tagIds = new HashSet<>();
        List<PoseObv> poseObservations = new LinkedList<>();

        for(PhotonPipelineResult result:camera.getAllUnreadResults()) {

        }
    }
}
