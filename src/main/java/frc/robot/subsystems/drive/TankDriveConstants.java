package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.component.MotorComponent;
import frc.lib.io.motor.rev.SparkBaseIO;
import frc.lib.io.motor.rev.SparkBaseSimIO;
import frc.lib.mechanismSim.RollerSim;
import frc.lib.mechanismSim.SimObject;
import frc.lib.util.ConfigUtil;
import frc.robot.IDs;
import frc.robot.Robot;

public class TankDriveConstants {
    public static Voltage maxVoltage = Units.Volts.of(10.0);
    public static double gearing = 5; // May be wrong, gearbox doesn't say
    public static Distance trackWidth = Units.Inches.of(17.582);
    public static Matrix<N3, N1> stdDevs = VecBuilder.fill(1000000.0, 1000000.0, 1000000.0);

    // Information about motors driving system 
    public static final DCMotor motor = DCMotor.getCIM(2); // Same for both sides

    /**
     * Gets a MotorIO for the system, returning a real one when actually running and a simulated one when running the simulation.
     */
    public static final MotorComponent<SparkBaseIO> getComponent(boolean isLeft) {
        return new MotorComponent<SparkBaseIO>(getMotorIO(isLeft));
    }

    /**
     * Gets a MotorIO for the system, returning a real one when actually running and a simulated one when running the simulation.
     */
    @SuppressWarnings("unchecked")
    public static final SparkBaseIO getMotorIO(boolean isLeft) {
        IDs mainID = isLeft ? IDs.TANK_DRIVE_LEFT_MAIN : IDs.TANK_DRIVE_RIGHT_MAIN;
        IDs followerID = isLeft ? IDs.TANK_DRIVE_LEFT_FOLLOWER : IDs.TANK_DRIVE_RIGHT_FOLLOWER;

        return Robot.isReal() 
            ? new SparkBaseIO(
                MotorType.kBrushed,
                getMainConfig(),
                mainID.id,
                Pair.of(followerID.id, false)
                )
            : new SparkBaseSimIO(
                getSimObject(),
                motor,
                MotorType.kBrushed,
                getMainConfig(),
                mainID.id,
                Pair.of(followerID.id, false)
            );
    }

    /** 
     * Get the configuration of the main motor
     */ 
    public static final SparkMaxConfig getMainConfig() {
        SparkMaxConfig config = ConfigUtil.getSafeMaxConfig(gearing);

        return config;    
    }

    public static final SimObject getSimObject() {
        FlywheelSim system = 
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    motor, 
                    0.01, 
                    gearing),
                motor
            );
        return new RollerSim(system);
    }

}
