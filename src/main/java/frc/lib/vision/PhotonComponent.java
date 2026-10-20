package frc.lib.vision;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.vision.VisionSubsystem.PoseEstimate;

public class PhotonComponent extends VisionComponent {
    private final PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private Supplier<Rotation2d> headingGetter;

    public PhotonComponent(String cameraName, AprilTagFieldLayout fieldLayout, Transform3d robotToCam, Supplier<Rotation2d> headingGetter) {
        super(cameraName);
        camera = new PhotonCamera(cameraName);
        estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        this.headingGetter = headingGetter;
    }

    @Override
    public void periodic() {
        // Provide heading data to pose estimator for better estimates. Done first to ensure updates gotten in super.periodic are using latest heading data
        estimator.addHeadingData(Timer.getFPGATimestamp(), headingGetter.get()); 

        super.periodic();
    }

    @Override
    public PoseEstimate updateLatestEstimate() {
        // Get all unread results
        List<PhotonPipelineResult> unreads = camera.getAllUnreadResults();
                
        if (unreads.size() == 0) { // If no unread results, return empty estimate
            return PoseEstimate.empty;
        }

        // Get latest unread results
        PhotonPipelineResult photonRaw = unreads.get(unreads.size() - 1);

        // Get photon estimate optional
        var photonEstimateOpt = estimator.estimateCoprocMultiTagPose(photonRaw);
        boolean exists = photonEstimateOpt.isPresent();

        if (!exists) { // Return empty estimate if estimate does not exist
            return PoseEstimate.empty;
        }
        var photonEstimate = photonEstimateOpt.get(); // Must exist because would return if not

        // Extract key features of photon estimate
        Pose2d pose = photonEstimate.estimatedPose.toPose2d();
        double timestampSeconds = photonRaw.getTimestampSeconds();
        var targets = photonEstimate.targetsUsed;
        int tagCount = targets.size();

        double totalArea = 0;
        double totalDist = 0;
        for (int i = 0; i < tagCount; i++) {
            totalArea += targets.get(i).area;
            totalDist += targets.get(i).bestCameraToTarget.getTranslation().getNorm();
        }
        double averageArea = totalArea / tagCount;
        double averageDistance = totalDist / tagCount;

        PoseEstimate estimate = new PoseEstimate(pose, timestampSeconds, tagCount, averageDistance, averageArea, exists);

        return estimate;
    }    
}
