package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface Camera {
    public static interface Simulator {
        public void update(Pose2d estimatedPose);

        public void reset(Pose2d pose);
    }

    public static class Result {
        public Pose2d pose;
        public Translation2d translationToTarget;
        public double timestamp, maxAmbiguity, maxDistance, minDistance;
        public boolean isNew, hasTarget;
        public int singleIDUsed;
    }

    public static class Specification {
        public Specification(int width, int height, Rotation2d diagonalFOV,
                double averageError, double stdDevError, double averageLatency, double stdDevLatency, double fps) {
            this.width = width;
            this.height = height;
            this.diagonalFOV = diagonalFOV;
            this.averageError = averageError;
            this.stdDevError = stdDevError;
            this.averageLatency = averageLatency;
            this.stdDevLatency = stdDevLatency;
            this.fps = fps;
        }

        public final int width, height;
        public final Rotation2d diagonalFOV;
        public final double averageError, stdDevError;
        public final double averageLatency, stdDevLatency;
        public final double fps;
    }

    public void updateResult(Result result);

    public void setReferencePose(Pose2d pose);

    public String getName();

    public Simulator createSimulator(Specification spec);
}
