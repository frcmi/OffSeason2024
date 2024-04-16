package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Constants.VisionConstants;
import frc.robot.vision.Camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private static class CameraData {
        public Camera camera;
        public Camera.Result result;
    }

    private final CameraData[] cameras;

    public VisionSubsystem() {
        cameras = new CameraData[VisionConstants.kCameras.length];

        try {
            var fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

            for (int i = 0; i < cameras.length; i++) {
                var desc = VisionConstants.kCameras[i];

                var data = new CameraData();
                data.camera = desc.createCamera(VisionConstants.kCameraOffsets[i], fieldLayout);
                data.result = new Camera.Result();

                cameras[i] = data;
            }
        } catch (Exception exc) {
            System.out.println("Failed to initialize Vision!");
        }
    }

    @Override
    public void periodic() {
        for (var camera : cameras) {
            camera.camera.updateResult(camera.result);
        }
    }

    public List<Camera.Result> getViableResults() {
        var results = new ArrayList<Camera.Result>();

        for (var camera : cameras) {
            var result = camera.result;
            if (!result.isNew) {
                continue;
            }

            if (result.maxDistance > VisionConstants.kMaxDistance || result.maxAmbiguity > VisionConstants.kMaxAmbiguity) {
                continue;
            }

            results.add(result);
        }

        return results;
    }
}