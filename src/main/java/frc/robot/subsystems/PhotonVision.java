import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PhotonVision extends SubsystemBase {
    //Arbitrary location– need to change later.
    public static final Transform3d cameraTransform = new Transform3d(
      new Translation3d(Units.inchesToMeters(10), 0, Units.inchesToMeters(10)), 
      new Rotation3d(0, 0, 0)
    );

    public Camera camera = new Camera("Camera", cameraTransform);

    public PhotonVision() {}

    @Override
    public void periodic() {

        camera.updateVision();
    }

    public void setReferencePose(Pose2d pose) {
        camera.setReferencePose(pose);
    }

    public Pose2d getCameraPose() {
        return camera.cameraPose;
    }

    public Double getCameraTimestamp() {
        return camera.cameraTimestamp;
    }

    public double getMinDistance() {
        return camera.getMinDistance();
    }

}