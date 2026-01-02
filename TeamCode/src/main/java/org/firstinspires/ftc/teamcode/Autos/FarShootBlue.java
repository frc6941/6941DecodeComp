package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.CloseShootCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveForTimeCommand;
import org.firstinspires.ftc.teamcode.Commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.LockHeadingCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.BeamBreakSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Utils.RobotStateRecoder;
import org.firstinspires.ftc.teamcode.display;

@Autonomous(name = "BLUE - Far Shoot", group = "Competition")
public class FarShootBlue extends CommandOpMode {

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private FeederSubsystem feeder;
    private LimelightSubsystem limelight;
    private BeamBreakSubsystem beamBreak;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        feeder = new FeederSubsystem(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap);
        beamBreak = new BeamBreakSubsystem(hardwareMap, "shooterBB", 1.0);

        schedule(new SequentialCommandGroup(
                new InstantCommand(
                        () -> {
                            RobotStateRecoder.setDriverAlliance(RobotStateRecoder.DriverAlliance.BLUE);
                            drive.resetHeading();
                            drive.applyDriverAlliance(RobotStateRecoder.DriverAlliance.BLUE);
                            drive.setPose(Constants.Field.BLUE_AUTO_START_POS);
                            drive.setFieldCentricEnabled(false);
                            shooter.setTargetRpm(3950);
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
                new CloseShootCommand(shooter, feeder, () -> 3950, 5000),
                new InstantCommand(
                        () -> {
                            drive.setFieldCentricEnabled(true);
                        }
                ),
                new LockHeadingCommand(
                        drive,
                        () -> 0,
                        () -> 0,
                        () -> RobotStateRecoder.getDriverAlliance() == RobotStateRecoder.DriverAlliance.BLUE
                                ? 90 : -90,
                        2.0,
                        telemetry
                ),
                new ParallelDeadlineGroup(
                        new IntakeCommand(feeder, () -> beamBreak.isBeamBreakOn(), 1, 0).withTimeout(5000),
                        new DriveForTimeCommand(
                                drive,
                                -0.3,
                                0.7,
                                0,
                                5
                        )
                ),

                new ParallelDeadlineGroup(
                        new IntakeCommand(feeder, () -> beamBreak.isBeamBreakOn(), 1, 0).withTimeout(1500),
                        new SequentialCommandGroup(
                                new DriveForTimeCommand(
                                        drive,
                                        -0.2,
                                        -0.5,
                                        0,
                                        0.5
                                ),
                                new DriveForTimeCommand(
                                        drive,
                                        -0.2,
                                        0.5,
                                        0,
                                        1
                                ))
                ),

                new ParallelDeadlineGroup(
                        new IntakeCommand(feeder, () -> beamBreak.isBeamBreakOn(), 1, 0).withTimeout(1500),
                        new SequentialCommandGroup(
                                new DriveForTimeCommand(
                                        drive,
                                        -0.2,
                                        -0.5,
                                        0,
                                        0.5
                                ),
                                new DriveForTimeCommand(
                                        drive,
                                        -0.2,
                                        0.5,
                                        0,
                                        1
                                ))
                ),
                new GoToPoseCommand(drive, Constants.Field.BLUE_AUTO_START_POS, 0.5).withTimeout(5000),
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
                new CloseShootCommand(shooter, feeder, () -> 3950, 5000),
                new GoToPoseCommand(drive, Constants.Field.STOP_BLUE_FAR, 0.5)
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
        telemetry.addData("BB", beamBreak.getVoltage());

        Pose2d pose = drive.getPose();
        Pose2d ghost = limelight.getPoseEstimate().orElse(null);
        display.sendPoseWithGhost(pose, ghost);
        telemetry.update();

    }
}