package org.firstinspires.ftc.teamcode._Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotDrive;
import org.firstinspires.ftc.teamcode._Libs.AutoOpMode;
import org.firstinspires.ftc.teamcode._Libs.RoadRunnerImplementer;

@Autonomous(name="Example", group="Autonomous")
public class TrajectoryTest extends AutoOpMode {
    Trajectory mTrajectory;
    RobotDrive mDrive;
    FtcDashboard mDashboard;

    @Override
    public void setup() {
        mDrive = new RobotDrive(hardwareMap);

        mDashboard = FtcDashboard.getInstance();

        // change these constraints to something reasonable for your drive
        mTrajectory = mDrive.trajectoryBuilder()
                .turnTo(Math.PI)
                .waitFor(2)
                .turnTo(0)
                .waitFor(2)
                .lineTo(new Vector2d(60, 0))
                .waitFor(2)
                .splineTo(new Pose2d(0, 40, 0))
                .build();

        mSequence.add(new RoadRunnerImplementer.Follow2dTrajectory(mDrive, mDashboard, mTrajectory));
    }
}