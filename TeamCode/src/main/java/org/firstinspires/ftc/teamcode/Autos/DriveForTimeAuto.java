package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.DriveForTimeCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

@Autonomous(name = "Auto Drive For Time", group = "Competition")
public class DriveForTimeAuto extends CommandOpMode {

    private DriveSubsystem drive;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new DriveSubsystem(hardwareMap, telemetry);
        register(drive);

        // 自动阶段只做“按摇杆坐标驱动”，避免 field-centric 依赖 IMU/里程计造成不确定性
        drive.setFieldCentricEnabled(false);
        drive.resetHeading();

        telemetry.addData("Auto", "Leave");
        telemetry.addData("Seconds", Constants.Auto.DRIVE_SECONDS);
        telemetry.addData("LX", Constants.Auto.DRIVE_LEFT_X);
        telemetry.addData("LY", Constants.Auto.DRIVE_LEFT_Y);
        telemetry.addData("RX", Constants.Auto.DRIVE_RIGHT_X);
        telemetry.update();

        schedule(new SequentialCommandGroup(
                new WaitCommand(35000),
                new DriveForTimeCommand(
                        drive,
                        Constants.Auto.DRIVE_LEFT_X,
                        Constants.Auto.DRIVE_LEFT_Y,
                        Constants.Auto.DRIVE_RIGHT_X,
                        Constants.Auto.DRIVE_SECONDS
                )
        ));
    }
}



