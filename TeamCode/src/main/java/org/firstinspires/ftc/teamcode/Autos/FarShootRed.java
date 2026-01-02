package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.CloseShootCommand;
import org.firstinspires.ftc.teamcode.Commands.CloseShootOpenLoopCommand;
import org.firstinspires.ftc.teamcode.Commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.Commands.LockHeadingCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AutoShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Utils.RobotStateRecoder;
import org.firstinspires.ftc.teamcode.display;

@Autonomous(name = "RED - Far Shoot", group = "Competition")
public class FarShootRed extends CommandOpMode {

    private DriveSubsystem drive;
    private AutoShooterSubsystem shooter;
    private FeederSubsystem feeder;
    private LimelightSubsystem limelight;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new AutoShooterSubsystem(hardwareMap, telemetry);
        feeder = new FeederSubsystem(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap);

        schedule(new SequentialCommandGroup(
                new InstantCommand(
                        () -> {
                            RobotStateRecoder.setDriverAlliance(RobotStateRecoder.DriverAlliance.RED);
                            drive.resetHeading();
                            drive.applyDriverAlliance(RobotStateRecoder.DriverAlliance.RED);
                            drive.setFieldCentricEnabled(false);
                        }
                ),
                new WaitCommand(2000),//vision sync
                //new GoToPoseCommand(drive, Constants.Field.GOAL_RED_CLOSE, 0.5).withTimeout(7000),
                new LockHeadingCommand(
                        drive,
                        () -> 0,
                        () -> 0,
                        () -> {
                            final Pose2d goal = RobotStateRecoder.getDriverAlliance() == RobotStateRecoder.DriverAlliance.BLUE
                                    ? Constants.Field.GOAL_BLUE
                                    : Constants.Field.GOAL_RED;
                            final Pose2d pose = drive.getPose();

                            final double dx = goal.getX() - pose.getX();
                            final double dy = goal.getY() - pose.getY();
                            return Math.toDegrees(Math.atan2(dy, dx));
                        },
                        2.0,
                        telemetry
                ),
                new CloseShootCommand(shooter, feeder, () -> 4000, 4000),
                new GoToPoseCommand(drive, Constants.Field.STOP_RED_FAR, 0.5).withTimeout(5000)
        ).andThen(
                new InstantCommand(
                        () -> {
                            drive.setFieldCentricEnabled(true);
                        }
                )
        ));
    }

    @Override
    public void run() {
        super.run();

        // Update LL with current gyro, then fuse vision if valid
        limelight.updateWithImuHeading(drive.getHeadingDegrees() - DriveSubsystem.GYRO_HEADING_OFFSET_DEG);
        limelight.getPoseEstimate().ifPresent(visionPose -> {
            final double distanceMeters = limelight.getDistanceMeters().orElse(Double.NaN);
            if (!Double.isNaN(distanceMeters)) {
                Pose2d fused = drive.fuseVisionPose(visionPose, distanceMeters);
                telemetry.addData("Vision Fused", true);
                telemetry.addData("Vision Gain", "auto");
            }
        });

        telemetry.addData("Driver Alliance", RobotStateRecoder.getDriverAlliance());
        telemetry.addData("shooting Position", RobotStateRecoder.getShootingPosition());
        telemetry.addData("Driver Input Offset (deg)", drive.getDriverInputOffsetDeg());
        telemetry.addData("Shooter Rpm", shooter.getVelocityRpm());

        Pose2d pose = drive.getPose();
        Pose2d ghost = limelight.getPoseEstimate().orElse(null);
        display.sendPoseWithGhost(pose, ghost);
        telemetry.update();

    }
}


