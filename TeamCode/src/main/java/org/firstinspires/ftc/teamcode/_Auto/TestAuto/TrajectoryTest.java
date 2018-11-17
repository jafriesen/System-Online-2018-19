package org.firstinspires.ftc.teamcode._Auto.TestAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._Libs.AutoOpMode;
import org.firstinspires.ftc.teamcode._Libs.RoadRunnerImplementer;
import org.firstinspires.ftc.teamcode.drive.SampleTankDriveREV;
import org.firstinspires.ftc.teamcode.drive.SampleTankDriveREVOptimized;

@Autonomous(name="Trajectory Test", group="Test")
public class TrajectoryTest extends AutoOpMode {
    Trajectory mTrajectory;
    SampleTankDriveREVOptimized mDrive;
    FtcDashboard mDashboard;

    @Override
    public void setup() {
        mDrive = new SampleTankDriveREVOptimized(hardwareMap);

        // change these constraints to something reasonable for your drive
        mTrajectory = mDrive.trajectoryBuilder()
                .splineTo(new Pose2d(12, 12, 0))
                .waitFor(1)
                .splineTo(new Pose2d(0, 0, 0))
                .build();

        mSequence.add(new RoadRunnerImplementer.Follow2dTrajectory(mDrive, mDashboard, mTrajectory));
    }
}