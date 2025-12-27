package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utils.RobotStateRecoder;

public class DriveSubsystem extends SubsystemBase {

    public static final double GYRO_HEADING_OFFSET_DEG = 180;
    // Pinpoint expects offsets in mm: x = sideways (left +), y = forward +
    private static final double PINPOINT_X_OFFSET_MM = 72;
    private static final double PINPOINT_Y_OFFSET_MM = -180.0;
    private final MecanumDrive drive;
    private final Telemetry telemetry;
    private GoBildaPinpointDriver pinpoint;
    private Pose2d rawPose = new Pose2d(0, 0, 0);
    private Pose2d previousRawPose = new Pose2d(0, 0, 0);
    private Pose2d poseOffset = new Pose2d(0, 0, 0);
    private boolean fieldCentricEnabled = Constants.Drive.DEFAULT_FIELD_CENTRIC;
    private double driverInputOffsetDeg = Constants.Drive.DRIVER_INPUT_OFFSET_BLUE_DEG;

    public DriveSubsystem(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this.telemetry = telemetry;
        final Motor frontLeft = buildDriveMotor(hardwareMap, Constants.Drive.FRONT_LEFT_NAME);
        final Motor frontRight = buildDriveMotor(hardwareMap, Constants.Drive.FRONT_RIGHT_NAME);
        final Motor backLeft = buildDriveMotor(hardwareMap, Constants.Drive.BACK_LEFT_NAME);
        final Motor backRight = buildDriveMotor(hardwareMap, Constants.Drive.BACK_RIGHT_NAME);

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // 可选：如果没装/没配置 PinPoint，就跳过（避免直接崩溃）
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "PinPoint");
            pinpoint.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.REVERSED,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            pinpoint = null;
            if (telemetry != null) {
                telemetry.addData("PinPoint", "NOT FOUND - odometry disabled");
            }
        }
    }

    private Motor buildDriveMotor(final HardwareMap hardwareMap, final String name) {
        final Motor motor = new Motor(hardwareMap, name, Motor.GoBILDA.RPM_435);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    @Override
    public void periodic() {
        if (pinpoint == null) {
            return;
        }
        pinpoint.update();
        final Pose2D p = pinpoint.getPosition();
        previousRawPose = rawPose;
        rawPose = new Pose2d(
                p.getX(DistanceUnit.INCH),
                p.getY(DistanceUnit.INCH),
                p.getHeading(AngleUnit.RADIANS)
        );

        // 计算位移增量
        double deltaX = rawPose.getX() - previousRawPose.getX();
        double deltaY = rawPose.getY() - previousRawPose.getY();
        double deltaHeading = rawPose.getHeading() - previousRawPose.getHeading();
        double deltaDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

//        // 记录到telemetry
//        if (telemetry != null) {
//            telemetry.addData("Pinpoint Delta X", "%.3f in", deltaX);
//            telemetry.addData("Pinpoint Delta Y", "%.3f in", deltaY);
//            telemetry.addData("Pinpoint Delta Distance", "%.3f in", deltaDistance);
//            telemetry.addData("Pinpoint Delta Heading", "%.3f rad", deltaHeading);
//        }
    }

    public void drive(final double leftX, final double leftY, final double rightX) {
        if (fieldCentricEnabled) {
            final double rotatedX;
            final double rotatedY;
            if (Math.abs(driverInputOffsetDeg) < 1e-9) {
                rotatedX = leftX;
                rotatedY = leftY;
            } else {
                final double[] rotated = rotateVector(leftX, leftY, driverInputOffsetDeg);
                rotatedX = rotated[0];
                rotatedY = rotated[1];
            }
            drive.driveFieldCentric(rotatedX, rotatedY, rightX, getHeadingDegrees(), false);
        } else {
            drive.driveRobotCentric(leftX, leftY, rightX, false);
        }
    }

    public double getDriverInputOffsetDeg() {
        return driverInputOffsetDeg;
    }

    /**
     * Set a driver-station perspective offset (degrees) applied to the
     * translation stick before field-centric transform. Typical usage: blue=0°,
     * red=180°.
     */
    public void setDriverInputOffsetDeg(final double offsetDeg) {
        driverInputOffsetDeg = offsetDeg;
    }

    public void stop() {
        drive.driveRobotCentric(0.0, 0.0, 0.0, false);
    }

    public Pose2d getPose() {
        return addOffset(rawPose, poseOffset);
    }

    public void setPose(final Pose2d pose) {
        Pose2d current = getPose();
        double dx = pose.getX() - current.getX();
        double dy = pose.getY() - current.getY();
        poseOffset = new Pose2d(poseOffset.getX() + dx, poseOffset.getY() + dy, 0.0);
    }

    /**
     * Blend the current Pinpoint pose with a vision pose using a distance-based
     * gain, then commit the fused pose. Returns the fused pose.
     */
    public Pose2d fuseVisionPose(final Pose2d visionPose, final double distanceMeters) {
        final Pose2d base = getPose();
        final double gain = computeVisionGain(distanceMeters);
        final double x = lerp(base.getX(), visionPose.getX(), gain);
        final double y = lerp(base.getY(), visionPose.getY(), gain);
        final Pose2d fused = new Pose2d(x, y, base.getHeading()); // keep IMU heading
        setPose(fused);
        return fused;
    }

    public double getHeadingDegrees() {
        double heading = Math.toDegrees(getPose().getHeading());
        heading += GYRO_HEADING_OFFSET_DEG; // 添加 gyro offset 修正旋转中心
        return heading;
    }

    public void resetHeading() {
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
        } else {
            zeroPoseEstimate();
        }
    }

    private void zeroPoseEstimate() {
        rawPose = new Pose2d(0, 0, 0);
        previousRawPose = new Pose2d(0, 0, 0);
        poseOffset = new Pose2d(0, 0, 0);
    }

    private double computeVisionGain(final double distanceMeters) {
        double gain = 0.35 / (distanceMeters + 0.5);
        return clamp(gain, 0.05, 0.6);
    }

    private double lerp(final double a, final double b, final double t) {
        return a + (b - a) * t;
    }

    private double angleWrap(final double angleRad) {
        double wrapped = angleRad;
        while (wrapped > Math.PI) {
            wrapped -= 2.0 * Math.PI;
        }
        while (wrapped < -Math.PI) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped;
    }

    private double clamp(final double value, final double min, final double max) {
        return Math.max(min, Math.min(max, value));
    }

    // Rotate (x,y) by +deg CCW
    private double[] rotateVector(final double x, final double y, final double deg) {
        final double rad = Math.toRadians(deg);
        final double cos = Math.cos(rad);
        final double sin = Math.sin(rad);
        final double rx = x * cos - y * sin;
        final double ry = x * sin + y * cos;
        return new double[]{rx, ry};
    }

    private Pose2d addOffset(final Pose2d base, final Pose2d offset) {
        return new Pose2d(
                base.getX() + offset.getX(),
                base.getY() + offset.getY(),
                angleWrap(base.getHeading() + offset.getHeading())
        );
    }

    public void setFieldCentricEnabled(boolean fieldCentricEnabled) {
        this.fieldCentricEnabled = fieldCentricEnabled;
    }

    public boolean isFieldCentricEnabled() {
        return fieldCentricEnabled;
    }

    public void applyDriverAlliance(final RobotStateRecoder.DriverAlliance alliance) {
        if (alliance == RobotStateRecoder.DriverAlliance.RED) {
            setDriverInputOffsetDeg(Constants.Drive.DRIVER_INPUT_OFFSET_RED_DEG);
        } else {
            setDriverInputOffsetDeg(Constants.Drive.DRIVER_INPUT_OFFSET_BLUE_DEG);
        }
    }
}
