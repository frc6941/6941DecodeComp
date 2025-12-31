package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

/**
 * 按固定时长驱动底盘（非阻塞，不会卡主线程）。
 */
public class DriveForTimeCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final double leftX;
    private final double leftY;
    private final double rightX;
    private final double seconds;

    public DriveForTimeCommand(
            final DriveSubsystem drive,
            final double leftX,
            final double leftY,
            final double rightX,
            final double seconds
    ) {
        this.drive = drive;
        this.leftX = clamp(leftX);
        this.leftY = clamp(leftY);
        this.rightX = clamp(rightX);
        this.seconds = Math.max(0.0, seconds);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        drive.drive(leftX, leftY, rightX);
    }

    @Override
    public void end(final boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= seconds;
    }

    private static double clamp(final double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }
}



