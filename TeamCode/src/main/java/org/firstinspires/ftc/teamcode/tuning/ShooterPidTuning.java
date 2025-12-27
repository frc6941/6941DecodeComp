package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * FTC Dashboard 可在线修改的 Shooter PID/FF 参数。 在 Dashboard 的 Config 页面里搜索本类名即可。
 */
@Config
public final class ShooterPidTuning {

    /**
     * 是否启用 Shooter RPM 闭环（调参OpMode会使用该开关）。
     */
    public static boolean ENABLED = true;

    /**
     * 目标飞轮速度（RPM）。
     */
    public static double TARGET_RPM = Constants.Shooter.TARGET_RPM;

    /**
     * PID: 输出为 [-1..1] 的功率修正量，输入为 RPM。
     */
    public static double kP = Constants.Shooter.VELOCITY_kP;
    public static double kI = Constants.Shooter.VELOCITY_kI;
    public static double kD = Constants.Shooter.VELOCITY_kD;

    /**
     * 前馈：power ≈ kF * targetRpm 如果你用纯 PID 也能收敛，可以先保持 0。
     */
    public static double kF = Constants.Shooter.VELOCITY_kF;

    /**
     * 输出功率限幅。
     */
    public static double MIN_POWER = Constants.Shooter.MIN_POWER;
    public static double MAX_POWER = Constants.Shooter.MAX_POWER;

    /**
     * 在“接近目标”判断时使用的 RPM 容差（用于 telemetry）。
     */
    public static double RPM_TOLERANCE = Constants.Shooter.RPM_TOLERANCE;

    private ShooterPidTuning() {
    }
}
