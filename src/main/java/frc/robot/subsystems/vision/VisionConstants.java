package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.vision.PhotonComponent;

public class VisionConstants {
    public static final String cameraName = "cam1";
    public static AprilTagFieldLayout fieldLayout;
    public static Transform3d robotToCam = new Transform3d(new Translation3d(), new Rotation3d());
    public static Matrix<N3, N1> stdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

    static {
        try {
            fieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/field_layout.json");
            SmartDashboard.putBoolean("Vision Found Field Layout: ", true);

        } catch (Exception e) {
            fieldLayout = null;
            SmartDashboard.putBoolean("Vision Found Field Layout: ", false);
        }
    }

    public static PhotonComponent getComponent(Supplier<Pose2d> drivePoseSupplier) {
        return new PhotonComponent(cameraName, fieldLayout, robotToCam, () -> drivePoseSupplier.get().getRotation());
    }
}
