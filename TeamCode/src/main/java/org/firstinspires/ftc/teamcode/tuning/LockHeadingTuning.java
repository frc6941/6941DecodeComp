package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.Constants;

@Config
public final class LockHeadingTuning {


    // 注意：Dashboard 调参要求字段不能是 final（否则不会实时生效）。
    public static double TURN_kP = Constants.Drive.LOCK_HEADING_TURN_kP;
    public static double TURN_kI = Constants.Drive.LOCK_HEADING_TURN_kI;
    public static double TURN_kD = Constants.Drive.LOCK_HEADING_TURN_kD;
    public static double TURN_MAX_OUTPUT = Constants.Drive.LOCK_HEADING_MAX_OUTPUT;

    public static double TARGET_DEGREE = Constants.Drive.LOCK_HEADING_TARGET_DEG;

    private LockHeadingTuning() {
    }
}
