package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Revolution;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Util.AprilTagPipelineSettings;
import frc.robot.generated.TunerConstants;

public class Constants {
    // Robot Constants
    public static final LinearVelocity maxLinVel = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity maxAngVel = RotationsPerSecond.of(2); 
    public static final AngularAcceleration maxAngAccel = DegreesPerSecondPerSecond.of(1292);
    public static final LinearAcceleration maxLinAccel = MetersPerSecondPerSecond.of(5.9);
    public static final LinearVelocity slowdownLinVel = maxLinVel.div(1.5);
    public static final AngularVelocity slowdownAngVel = RotationsPerSecond.of(1.5);
    public static final Distance linRampDownDist = Meters.of(1.2);

    public static final Distance robotWidth = Inches.of(29.5);
    public static final Distance robotLength = Inches.of(29.5);
    public static final Distance robotWithBumpersWidth = Inches.of(36.5);//TODO CHANGE
    public static final Distance robotWithBumpersLength = Inches.of(29.5);//TODO CHANGE
    public static final Distance bumperThickness = Inches.of(3);

    public static final Current krakenX60CurrentLimit = Amps.of(80);

    public static final CANBus rioBus = CANBus.roboRIO();
    public static final CANBus canivoreBus = new CANBus("canivore");

    // Motor IDs

    //Other CAN IDs

    // Drivetrain Constants
    public static final LinearVelocity linDeadband = maxLinVel.times(.07); 
    public static final AngularVelocity angDeadband = maxAngVel.times(.07);

    // Camera Constants
    public static final Transform3d leftCameraOffsets = new Transform3d(
        Inches.of(-11.274), 
        Inches.of(13.476), 
        Inches.of(7.553), 
        new Rotation3d(0, Math.toRadians(-40), Math.toRadians(-130))
    );

    public static final Transform3d rightCameraOffsets = new Transform3d(
        Inches.of(-11.274), 
        Inches.of(-13.476), 
        Inches.of(7.553), 
        //new Rotation3d(37.8, 37.8, -45)
        new Rotation3d(0, Math.toRadians(-40), Math.toRadians(130))
    );

    public static final Transform3d sideCameraOffsets = new Transform3d(
         Inches.of(-9.074),    
        Inches.of(15.228), 
        Inches.of(12.205), 
        //new Rotation3d(37.8, 37.8, -45)
        new Rotation3d(0, Math.toRadians(0), Math.toRadians(90))
    );

    public static final Vector<N3> singleStds = VecBuilder.fill(1.5, 1.5, 32);
    public static final Vector<N3> multiStds = VecBuilder.fill(0.5, 0.5, 1);

    public static final AprilTagPipelineSettings leftCameraSettings = new AprilTagPipelineSettings(AprilTagFields.k2026RebuiltWelded,
        Constants.leftCameraOffsets,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        4, 
        singleStds , 
        multiStds,
        .2
    );

    public static final AprilTagPipelineSettings rightCameraSettings = new AprilTagPipelineSettings(AprilTagFields.k2026RebuiltWelded,
        Constants.rightCameraOffsets,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        4, 
        singleStds , 
        multiStds,
        .2
    );

    //Distance Constants
    public static final Distance maxRobotTrenchDistance = Meters.of(2);
    public static final Distance autonPointToPointTolerance = Meters.of(0.1);
}
