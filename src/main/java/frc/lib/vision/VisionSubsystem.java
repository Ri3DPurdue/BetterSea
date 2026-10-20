package frc.lib.vision;

import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public class VisionSubsystem extends SubsystemBase implements Loggable{
    private final VisionComponent[] cameras;
    private final BiConsumer<Pose2d, Double> visionUpdater;

    public VisionSubsystem(BiConsumer<Pose2d, Double> addVisionEstimate, VisionComponent... cameras) {
        this.cameras = cameras;
        visionUpdater = addVisionEstimate;
    }

    @Override
    public void periodic() {
        for (VisionComponent camera : cameras) {
            camera.periodic();
        }
        addLatestEstimate();
    }

    /**
     * 
     * @param estimate A PoseEstimate to evaluate if is acceptable
     * @return Whether estimate is acceptable enough to utilize, currently just if the estimate actually exists
     */
    public boolean shouldAcceptEstimate(PoseEstimate estimate) {
        return estimate.exists();
    }

    /**
     * Add the latest vision estimate to the drivetrain for each camera
     */
    public void addLatestEstimate() {
        for (VisionComponent camera : cameras) {
            PoseEstimate latestEstimate = camera.getLatestEstimate();
            if (shouldAcceptEstimate(latestEstimate)) {
                visionUpdater.accept(latestEstimate.pose(), latestEstimate.timestampSeconds());
            }
        }
    }


    @Override
    public void log(String path) {
        for (VisionComponent camera : cameras) {
            Logger.log(path, camera);
        }

    }

    /** Holds an estimated position from a vison system. */
    public static record PoseEstimate (
        Pose2d pose,
        double timestampSeconds,
        int tagCount,
        double averageDistance,
        double averageArea,
        boolean exists) {
            public static final PoseEstimate empty = new PoseEstimate(new Pose2d(), 0, 0, 0, 0, false);
        }
}
