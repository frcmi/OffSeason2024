package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraDescription {
    public static enum CameraType {
        PHOTONVISION
    }

    /**
      * Describes a camera to connect to for position estimation
      * @param name Name of the camera
      * @param type Backend to use
      */
    public CameraDescription(String name, CameraType type) {
        cameraName = name;
        cameraType = type;
    }

    public Camera createCamera(Transform3d robotOffset, AprilTagFieldLayout layout) {
        switch (cameraType) {
            case PHOTONVISION -> {
                return new PhotonlibCamera(cameraName, robotOffset, layout);
            }
            default -> {
                return null;
            }
        }
    }

    private final String cameraName;
    private final CameraType cameraType;
}
