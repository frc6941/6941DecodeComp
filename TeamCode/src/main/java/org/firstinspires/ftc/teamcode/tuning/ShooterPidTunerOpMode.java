package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

/**
 * Shooter PID 调参专用 OpMode：
 * - 在 Dashboard 的 Config 里修改 {@link ShooterPidTuning} 的参数（目标RPM + PID/FF）。
 * - 在 Dashboard 图表里查看 targetRpm / actualRpm / power 的时间曲线。
 */
@TeleOp(name = "Tuning: Shooter PID (Dashboard)", group = "Tuning")
public class ShooterPidTunerOpMode extends CommandOpMode {

    private ShooterSubsystem shooter;
    private FtcDashboard dashboard;

    @Override
    public void initialize() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        shooter = new ShooterSubsystem(hardwareMap);
        register(shooter);

        shooter.closeLatch();
        telemetry.addLine("Use Dashboard Config: ShooterPidTuning");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        if (ShooterPidTuning.ENABLED) {
            shooter.setVelocityRpm(ShooterPidTuning.TARGET_RPM);
            shooter.closeLatch();
        } else {
            shooter.stop();
        }

        final TelemetryPacket packet = new TelemetryPacket();
        packet.put("shooter/targetRpm", shooter.getTargetRpm());
        packet.put("shooter/actualRpm", shooter.getVelocityRpm());
        packet.put("shooter/errorRpm", shooter.getVelocityErrorRpm());
        packet.put("shooter/power", shooter.getLastAppliedPower());
        packet.put("shooter/closedLoop", shooter.isVelocityClosedLoopEnabled());
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Target RPM", shooter.getTargetRpm());
        telemetry.addData("Actual RPM", shooter.getVelocityRpm());
        telemetry.addData("Error RPM", shooter.getVelocityErrorRpm());
        telemetry.addData("Power", shooter.getLastAppliedPower());
        telemetry.addData("ClosedLoop", shooter.isVelocityClosedLoopEnabled());
        telemetry.addData("AtTarget", shooter.atTargetRpm());
        telemetry.update();
    }
}



