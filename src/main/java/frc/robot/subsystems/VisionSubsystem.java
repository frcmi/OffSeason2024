package frc.robot.subsystems;

import java.util.HashMap;

import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import frc.robot.logging.BooleanLog;
import frc.robot.logging.DoubleLog;
import frc.robot.logging.StructLog;
import frc.robot.vision.Camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private static class CameraData {
        public Camera camera;
        public Camera.Result result;
        public Camera.Simulator sim;
        public boolean isViable;

        public StructLog<Pose2d> poseLog;
        public DoubleLog ambiguityLog, maxDistanceLog, minDistanceLog;
        public BooleanLog isViableLog;
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
                data.sim = null;
                data.isViable = false;

                data.poseLog = new StructLog<>("Vision/Pose " + i, Pose2d.struct);
                data.ambiguityLog = new DoubleLog("Vision/Ambiguity " + i);
                data.maxDistanceLog = new DoubleLog("Vision/Max distance " + i);
                data.minDistanceLog = new DoubleLog("Vision/Min distance " + i);
                data.isViableLog = new BooleanLog("Vision/Is pose " + i + " current?");

                if (Robot.isSimulation()) {
                    data.sim = data.camera.createSimulator(VisionConstants.kCameraSpecs[i]);
                }

                cameras[i] = data;
            }
        } catch (Exception exc) {
            System.out.println("Failed to initialize Vision!");
        }
    }

    @Override
    public void periodic() {
        for (var camera : cameras) {
            var result = camera.result;
            camera.camera.updateResult(result);

            camera.isViable = isResultViable(result);
            camera.isViableLog.update(camera.isViable);

            if (camera.isViable) {
                camera.poseLog.update(result.pose);
                camera.ambiguityLog.update(result.maxAmbiguity);
                camera.maxDistanceLog.update(result.maxDistance);
                camera.minDistanceLog.update(result.minDistance);
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        // todo: update with swerve
        var pose = new Pose2d(0, 0, new Rotation2d(0));

        for (var camera : cameras) {
            camera.sim.update(pose);
        }
    }

    public void resetSim(Pose2d pose) {
        for (var camera : cameras) {
            camera.sim.reset(pose);
        }
    }

    private boolean isResultViable(Camera.Result result) {
        if (!result.isNew) {
            return false;
        }

        if (result.maxDistance > VisionConstants.kMaxDistance || result.maxAmbiguity > VisionConstants.kMaxAmbiguity) {
            return false;
        }

        return true;
    }

    public HashMap<Integer, Camera.Result> getViableResults() {
        var results = new HashMap<Integer, Camera.Result>();

        for (int i = 0; i < cameras.length; i++) {
            var camera = cameras[i];
            if (!camera.isViable) {
                continue;
            }

            results.put(i, camera.result);
        }

        return results;
    }
}