package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * 测试编码器 Pod 方向的 OpMode
 *
 * 使用方法： 1. 启动 OpMode 2. 按 START 键重置位置到 (0, 0, 0) 3. 手动推动机器人： - 向前推：Y 坐标应该增加（正值）
 * - 向后推：Y 坐标应该减少（负值） - 向左推：X 坐标应该增加（正值） - 向右推：X 坐标应该减少（负值） 4. 如果方向相反，需要修改
 * DriveSubsystem 中的 setEncoderDirections
 */
@TeleOp(name = "测试: 编码器方向", group = "测试")
public class TestOdometryDirections extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;
    private Pose2D lastPose = null;

    @Override
    public void runOpMode() {
        // 初始化 Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "PinPoint");

        // 使用与 DriveSubsystem 相同的配置
        pinpoint.setOffsets(-0.05, -180.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        telemetry.addLine("=== 编码器方向测试 ===");
        telemetry.addLine("按 START 键重置位置");
        telemetry.addLine("");
        telemetry.addLine("测试步骤：");
        telemetry.addLine("1. 向前推机器人 → Y 应该增加");
        telemetry.addLine("2. 向后推机器人 → Y 应该减少");
        telemetry.addLine("3. 向左推机器人 → X 应该增加");
        telemetry.addLine("4. 向右推机器人 → X 应该减少");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 检查重置按键
            if (gamepad1.start) {
                pinpoint.resetPosAndIMU();
                lastPose = null;
                telemetry.addLine("位置已重置！");
            }

            // 更新 Pinpoint
            pinpoint.update();
            Pose2D currentPose = pinpoint.getPosition();

            // 显示当前位置
            telemetry.addLine("=== 当前位置 ===");
            telemetry.addData("X (英寸)", "%.2f", currentPose.getX(DistanceUnit.INCH));
            telemetry.addData("Y (英寸)", "%.2f", currentPose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading (度)", "%.2f", currentPose.getHeading(AngleUnit.DEGREES));
            telemetry.addLine("");

            // 显示变化量
            if (lastPose != null) {
                double dx = currentPose.getX(DistanceUnit.INCH) - lastPose.getX(DistanceUnit.INCH);
                double dy = currentPose.getY(DistanceUnit.INCH) - lastPose.getY(DistanceUnit.INCH);

                telemetry.addLine("=== 变化量 (自上次更新) ===");
                telemetry.addData("ΔX (英寸)", "%.3f", dx);
                telemetry.addData("ΔY (英寸)", "%.3f", dy);

                // 判断方向
                if (Math.abs(dx) > 0.01) {
                    if (dx > 0) {
                        telemetry.addLine("→ 机器人向左移动 ✓");
                    } else {
                        telemetry.addLine("→ 机器人向右移动 ✓");
                    }
                }
                if (Math.abs(dy) > 0.01) {
                    if (dy > 0) {
                        telemetry.addLine("→ 机器人向前移动 ✓");
                    } else {
                        telemetry.addLine("→ 机器人向后移动 ✓");
                    }
                }
            }

            telemetry.addLine("");
            telemetry.addLine("按 START 重置位置");
            telemetry.update();

            lastPose = currentPose;
            sleep(50); // 降低更新频率，便于观察
        }
    }
}
