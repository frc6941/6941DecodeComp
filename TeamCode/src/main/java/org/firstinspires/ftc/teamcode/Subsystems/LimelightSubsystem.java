package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

/**
 * Limelight3A subsystem using MT2 (vision + gyro fusion). Call
 * updateWithImuHeading() each loop before consuming pose to keep LL aligned to
 * the current yaw.
 */
public class LimelightSubsystem extends SubsystemBase {

    private static final double M_TO_IN = 39.37007874;

    private final Limelight3A limelight;
    private LLResult latest = null;

    public LimelightSubsystem(final HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
        limelight.start();
    }

    /**
     * Update Limelight with the current gyro heading (degrees, CCW+) and pull
     * the latest result.
     */
    public void updateWithImuHeading(final double headingDegrees) {
        limelight.updateRobotOrientation(headingDegrees);
        latest = limelight.getLatestResult();
    }

    public OptionalInt getTargetId() {
        if (latest != null && latest.isValid()) {
            final List<LLResultTypes.FiducialResult> fiducials = latest.getFiducialResults();
            if (!fiducials.isEmpty()) {
                return OptionalInt.of(fiducials.get(0).getFiducialId());
            }
        }
        return OptionalInt.empty();
    }

    /**
     * Robot pose estimate from Limelight MT2, returned in inches and radians
     * (RR Pose2d).
     */
    public Optional<Pose2d> getPoseEstimate() {
        if (latest != null && latest.isValid()) {
            Pose3D botpose = latest.getBotpose_MT2();
            if (botpose == null) {
                botpose = latest.getBotpose_MT2();
            }
            if (botpose != null) {
                final double xIn = botpose.getPosition().x * M_TO_IN;
                final double yIn = botpose.getPosition().y * M_TO_IN;
                // Limelight orientation is reported in degrees; convert to radians for RR.
                final double headingRad = Math.toRadians(botpose.getOrientation().getYaw());
                return Optional.of(new Pose2d(xIn, yIn, headingRad));
            }
        }
        return Optional.empty();
    }

    /**
     * Distance from robot origin to target in meters using MT2 pose if
     * available.
     */
    public OptionalDouble getDistanceMeters() {
        if (latest != null && latest.isValid()) {
            Pose3D targetPoseRobot = latest.getBotpose_MT2();
            if (targetPoseRobot == null) {
                targetPoseRobot = latest.getBotpose_MT2();
            }
            if (targetPoseRobot != null) {
                final double x = targetPoseRobot.getPosition().x;
                final double y = targetPoseRobot.getPosition().y;
                final double z = targetPoseRobot.getPosition().z;
                final double distance = Math.sqrt(x * x + y * y + z * z);
                return OptionalDouble.of(distance);
            }
        }
        return OptionalDouble.empty();
    }
}
