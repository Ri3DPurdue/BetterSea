package frc.robot.subsystems.vision;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.vision.VisionSubsystem;

public class Vision extends VisionSubsystem {    
    public Vision(BiConsumer<Pose2d, Double> addVisionEstimate, Supplier<Pose2d> drivePoseSupplier) {
        super(addVisionEstimate, VisionConstants.getComponent(drivePoseSupplier));
    }
}
