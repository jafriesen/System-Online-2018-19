package org.firstinspires.ftc.teamcode._Libs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleTankDriveREV;
import org.firstinspires.ftc.teamcode.drive.SampleTankDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

public class RoadRunnerImplementer {
    static public class Follow2dTrajectory extends AutoLib.Step {
        TankPIDVAFollower mFollower;
        Trajectory mTrajectory;
        SampleTankDriveREV mDrive;
        AutoOpMode mOpMode;

        public Follow2dTrajectory(AutoOpMode opMode, SampleTankDriveREV drive, Trajectory trajectory) {
            mOpMode = opMode;
            mDrive = drive;
            mTrajectory = trajectory;
        }

        public void setTrajectory(Trajectory trajectory) {
            mTrajectory = trajectory;
        }

        public boolean loop()
        {
            super.loop();

            if(firstLoopCall()) {

                mDrive.followTrajectory(mTrajectory);
            }

            Pose2d currentPose = mDrive.getPoseEstimate();

            Canvas fieldOverlay = mOpMode.packet.fieldOverlay();

            mOpMode.packet.put("x", currentPose.getX());
            mOpMode.packet.put("y", currentPose.getY());
            mOpMode.packet.put("heading", currentPose.getHeading());

            fieldOverlay.setStroke("green");
            DashboardUtil.drawSampledTrajectory(fieldOverlay, mTrajectory);

            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

            mDrive.update();

            return !mDrive.isFollowingTrajectory();
        }
    }
}
