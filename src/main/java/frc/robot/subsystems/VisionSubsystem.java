package frc.robot.subsystems;

import java.util.HashMap;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.logging.BooleanLog;
import frc.robot.logging.DoubleLog;
import frc.robot.logging.StructLog;
import frc.robot.vision.Camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Uses camera video input to judge position from April Tags
 */
public class VisionSubsystem extends SubsystemBase {
    /*
     * All data related to a camera on the bot
     */
    private static class CameraData {
        public Camera camera; // actual camera handle
        public Camera.Result result; // retrieved vision data
        public Camera.Simulator sim; // simulation handle
        public boolean isViable; // is this result good? is it stale?

        // all publishers for telemetry
        public StructLog<Pose2d> poseLog;
        public DoubleLog ambiguityLog, maxDistanceLog, minDistanceLog;
        public BooleanLog isViableLog;
    }

    private final CameraData[] cameras;

    public VisionSubsystem() {
        cameras = new CameraData[VisionConstants.kCameras.length];

        try {
            // read field layout from embedded frc data
            var fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

            for (int i = 0; i < cameras.length; i++) {
                var desc = VisionConstants.kCameras[i];

                // initialize camera
                var data = new CameraData();
                data.camera = desc.createCamera(VisionConstants.kCameraOffsets[i], fieldLayout);
                data.result = new Camera.Result();
                data.sim = null;
                data.isViable = false;

                // set up networktables logs
                data.poseLog = new StructLog<>("Vision/Pose " + i, Pose2d.struct);
                data.ambiguityLog = new DoubleLog("Vision/Ambiguity " + i);
                data.maxDistanceLog = new DoubleLog("Vision/Max distance " + i);
                data.minDistanceLog = new DoubleLog("Vision/Min distance " + i);
                data.isViableLog = new BooleanLog("Vision/Is pose " + i + " current?");

                if (Robot.isSimulation()) {
                    // create sim handler
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
            // read data from camera (pass-by-reference)
            var result = camera.result;
            camera.camera.updateResult(result);

            // validate result data
            camera.isViable = isResultViable(result);
            camera.isViableLog.update(camera.isViable);

            // if viable, log to networktables
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
        var swerve = RobotContainer.swerveSubsystem;
        var pose = swerve.getState().Pose;

        // generate images to feed to camera sim
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