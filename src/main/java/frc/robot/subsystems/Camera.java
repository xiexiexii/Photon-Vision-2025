package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera{
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final String name;
    private final PhotonCamera camera;
    private List<PhotonTrackedTarget> targets;
    private final PhotonPoseEstimator poseEstimator;

    public Pose2d cameraPose;
    public double cameraTimestamp;
    public double minDistance;
    public double targetYaw;

    public Camera(String name, Transform3d robotToCamera) {
        this.name = name;
        camera = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void updateVision() {
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            Optional<EstimatedRobotPose> currentPose = poseEstimator.update(result);
            if (currentPose.isPresent()) {
                if (result.hasTargets())
                    targets = currentPose.get().targetsUsed;
                if (targets != null) {
                    if (targets.size() > 1) {
                        double minDistance = Double.MAX_VALUE;
                        for (PhotonTrackedTarget target : targets) {
                            double distance = target.getBestCameraToTarget().getTranslation().getNorm();
                            if (distance < minDistance) 
                                minDistance = distance;
                        }
                        this.minDistance = minDistance;
                    } 
                    else 
                        minDistance = targets.get(0).getBestCameraToTarget().getTranslation().getNorm();
                        targetYaw=targets.get(0).getYaw();
                }
    
                cameraPose = new Pose2d(
                    currentPose.get().estimatedPose.getX(),
                    currentPose.get().estimatedPose.getY(),
                    currentPose.get().estimatedPose.getRotation().toRotation2d()
                );
    
                cameraTimestamp = currentPose.get().timestampSeconds;
    
            }

        }

    }

    public String getName() {
        return name;
    }

    public void setReferencePose(Pose2d pose) {
        poseEstimator.setReferencePose(pose);
    }

    public double getMinDistance() {
        return minDistance;
    }
}
