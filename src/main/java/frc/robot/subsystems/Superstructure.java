package frc.robot.subsystems;

import frc.robot.subsystems.drive.TankDrive;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.Vision;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public class Superstructure implements Loggable {
    // public final Drive drive = new Drive();
    public final TankDrive tankDrive = new TankDrive();
    public final Vision vision = new Vision(tankDrive::addVisionMeasurement, () -> tankDrive.getPose());
    public final Intake intake = new Intake();

    public final Command intake() {
        return intake.intake();
    }

    public final Command stow() {
        return intake.stow();
    }

    @Override
    public void log(String path) {
        // Logger.log(path, "Drive", drive);
       Logger.log(path, "Tank Drive", tankDrive);
       Logger.log(path, "Vision", vision);
       Logger.log(path, "Intake", intake);
    }
}
