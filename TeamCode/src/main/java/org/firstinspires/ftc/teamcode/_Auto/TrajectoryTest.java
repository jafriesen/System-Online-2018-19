package org.firstinspires.ftc.teamcode._Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._Libs.AutoOpMode;
import org.firstinspires.ftc.teamcode._Libs.RoadRunnerImplementer;
import org.firstinspires.ftc.teamcode.drive.SampleTankDriveREV;

@Autonomous(name="Trajectory Test", group="Autonomous")
public class TrajectoryTest extends AutoOpMode {
    Trajectory mTrajectory;
    SampleTankDriveREV mDrive;
    FtcDashboard mDashboard;

    @Override
    public void setup() {
        mDrive = new SampleTankDriveREV(hardwareMap);

        mDashboard = FtcDashboard.getInstance();

        // change these constraints to something reasonable for your drive
        mTrajectory = mDrive.trajectoryBuilder()
                .splineTo(new Pose2d(13, 12, 0))
                .waitFor(1)
                .splineTo(new Pose2d(0, 0, 0))
                .build();

        mSequence.add(new RoadRunnerImplementer.Follow2dTrajectory(mDrive, mDashboard, mTrajectory));
    }
}