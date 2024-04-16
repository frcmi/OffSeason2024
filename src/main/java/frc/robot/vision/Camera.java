package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface Camera {
    public static class Result {
        public Pose2d pose;
        public Translation2d translationToTarget;
        public double timestamp, maxAmbiguity, maxDistance, minDistance;
        public boolean isNew, hasTarget;
        public int singleIDUsed;
    }

    public void updateResult(Result result);
    public void setReferencePose(Pose2d pose);
    public String getName();
}
