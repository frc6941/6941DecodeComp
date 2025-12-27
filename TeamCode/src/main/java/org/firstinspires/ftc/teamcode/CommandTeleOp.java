package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CloseShootCommand;
import org.firstinspires.ftc.teamcode.Commands.IndexCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.LockHeadingCommand;
import org.firstinspires.ftc.teamcode.Commands.PreShootCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp(name = "TeleOp Drive (Command)", group = "Competition")
public class CommandTeleOp extends CommandOpMode {

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private FeederSubsystem feeder;
    private GamepadEx driverRC;
    private LimelightSubsystem limelight;
    private DriverAlliance driverAlliance = DriverAlliance.BLUE;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        feeder = new FeederSubsystem(hardwareMap);
        driverRC = new GamepadEx(gamepad1);
        limelight = new LimelightSubsystem(hardwareMap);

        applyDriverAlliance(DriverAlliance.BLUE);
        new GamepadButton(driverRC, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> applyDriverAlliance(DriverAlliance.RED));
        new GamepadButton(driverRC, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> applyDriverAlliance(DriverAlliance.BLUE));

        new GamepadButton(driverRC, GamepadKeys.Button.START)
                .whenPressed(drive::resetHeading);

        final Trigger leftTrigger = new Trigger(
                () -> driverRC.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
        );
        final Trigger rightTrigger = new Trigger(
                () -> driverRC.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
        );
        final Trigger leftBumper = new Trigger(
                () -> driverRC.getButton(GamepadKeys.Button.LEFT_BUMPER)
        );
        final Trigger rightBumper = new Trigger(
                () -> driverRC.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        );
        final Trigger buttonA = new Trigger(
                () -> driverRC.getButton(GamepadKeys.Button.A)
        );

        final Trigger buttonX = new Trigger(
                () -> driverRC.getButton(GamepadKeys.Button.X)
        );

        rightTrigger.whileActiveOnce(
                new CloseShootCommand(shooter, feeder, rightBumper)
        );
        leftTrigger.whileActiveContinuous(
                new IntakeCommand(feeder)
        );
        leftBumper.whileActiveContinuous(
                new IndexCommand(feeder, -Constants.Feeder.DEFAULT_INTAKE_POWER, -Constants.Feeder.DEFAULT_INDEX_POWER)
        );

        buttonX.whileActiveContinuous(
                new IndexCommand(feeder, 0, Constants.Feeder.DEFAULT_INDEX_POWER)
        );

        rightTrigger.whileActiveContinuous(
                new LockHeadingCommand(
                        drive,
                        driverRC::getLeftX,
                        driverRC::getLeftY,
                        () -> {
                            final Pose2d goal = driverAlliance == DriverAlliance.BLUE
                                    ? Constants.Field.GOAL_BLUE
                                    : Constants.Field.GOAL_RED;
                            final Pose2d pose = drive.getPose();

                            final double dx = goal.getX() - pose.getX();
                            final double dy = goal.getY() - pose.getY();
                            return Math.toDegrees(Math.atan2(dy, dx));
                        },
                        2.0,
                        telemetry
                )
        );
        register(drive);
        register(shooter);
        register(feeder);
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.drive(
                                driverRC.getLeftX(), // right +
                                driverRC.getLeftY(), // forward +
                                -driverRC.getRightX() // ccw + (left+)
                        ),
                        drive
                )
        );
        shooter.setDefaultCommand(
                new PreShootCommand(shooter)
        );
        feeder.setDefaultCommand(
                new RunCommand(
                        () -> feeder.stop(),
                        feeder
                )
        );

//        buttonA.whileActiveOnce(new GoToPoseCommand(
//                drive,
//                driverAlliance == DriverAlliance.BLUE ? Constants.Field.GOAL_BLUE_FRONT : Constants.Field.GOAL_RED_FRONT,
//                1));

    }

    @Override
    public void run() {
        super.run();
        driverRC.readButtons();

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

        telemetry.addData("LX(right +)", driverRC.getLeftX());
        telemetry.addData("LY(fwd +)", driverRC.getLeftY());
        telemetry.addData("RX(ccw +)", -driverRC.getRightX());
        telemetry.addData("Driver Alliance", driverAlliance);
        telemetry.addData("Driver Input Offset (deg)", drive.getDriverInputOffsetDeg());
        telemetry.addData("Shooter Rpm", shooter.getVelocityRpm());
//        telemetry.addData("Leader Rpm", shooter.getLeaderVelocityRpm());
//        telemetry.addData("Follower Rpm", shooter.getFollowerVelocityRpm());
//        telemetry.addData("Shooter Rps", shooter.getVelocityRps());
//        telemetry.addData("Shooter tpsEst", shooter.getDebugTicksPerSec());
//        telemetry.addData("Shooter targetRps", shooter.getTargetRps());
//        telemetry.addData("Shooter closedLoop", shooter.isVelocityClosedLoopEnabled());

        Pose2d pose = drive.getPose();
        Pose2d ghost = limelight.getPoseEstimate().orElse(null);
        display.sendPoseWithGhost(pose, ghost);
        telemetry.update();
    }

    private void applyDriverAlliance(final DriverAlliance alliance) {
        driverAlliance = alliance;
        if (alliance == DriverAlliance.RED) {
            drive.setDriverInputOffsetDeg(Constants.Drive.DRIVER_INPUT_OFFSET_RED_DEG);
        } else {
            drive.setDriverInputOffsetDeg(Constants.Drive.DRIVER_INPUT_OFFSET_BLUE_DEG);
        }
    }

    private enum DriverAlliance {
        BLUE,
        RED
    }
}
