package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

<<<<<<< HEAD
import org.firstinspires.ftc.teamcode.drive.SampleTankDriveREV;
import org.firstinspires.ftc.teamcode.drive.SampleTankDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
=======
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREV;
>>>>>>> 93786f9fe8062a74a4ad578e29cbd07342eac336

/*
 * Op mode for tuning follower PID coefficients. This is the final step in the tuning process.
 */
<<<<<<< HEAD
@Autonomous(name="FollowerPIDTuner", group="RR")
public class FollowerPIDTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        SampleTankDriveREV drive = new SampleTankDriveREV(hardwareMap);
=======
@Autonomous(group = "drive")
public class FollowerPIDTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
>>>>>>> 93786f9fe8062a74a4ad578e29cbd07342eac336

        drive.setPoseEstimate(new Pose2d(-24, -24, 0));

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(48)
                            .build()
            );
            drive.turnSync(Math.toRadians(90));
        }
    }
}
