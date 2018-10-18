package org.firstinspires.ftc.teamcode._Libs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotDrive;
import org.firstinspires.ftc.teamcode._RoadRunner.DashboardUtil;

public class RoadRunnerImplementer {
    public class FollowTrajectory extends AutoLib.Step {
        TankPIDVAFollower mFollower;
        Trajectory mTrajectory;
        RobotDrive mDrive;
        FtcDashboard mDashboard;

        public FollowTrajectory(HardwareMap hardwareMap, FtcDashboard dashboard, Trajectory trajectory) {
            mDrive = new RobotDrive(hardwareMap);
            // change these constraints to something reasonable for your drive
            DriveConstraints baseConstraints = new DriveConstraints(20.0, 30.0, Math.PI / 2, Math.PI / 2);
            TankConstraints constraints = new TankConstraints(baseConstraints, mDrive.getTrackWidth());
            // change these constraints to something reasonable for your drive
            mTrajectory = trajectory = mDrive.trajectoryBuilder()
                    .turnTo(Math.PI)
                    .waitFor(2)
                    .turnTo(0)
                    .waitFor(2)
                    .lineTo(new Vector2d(60, 0))
                    .waitFor(2)
                    .splineTo(new Pose2d(0, 40, 0))
                    .build();

            // TODO: tune kV, kA, and kStatic in the following follower
            // then tune the PID coefficients after you verify the open loop response is roughly correct
            mFollower = new TankPIDVAFollower(
                    mDrive,
                    new PIDCoefficients(0, 0, 0),
                    new PIDCoefficients(0, 0, 0),
                    0,
                    0,
                    0);
        }

        public boolean loop()
        {
            super.loop();

            if(firstLoopCall()) {
                mDrive.followTrajectory(mTrajectory);
            }

            Pose2d currentPose = mDrive.getPoseEstimate();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            packet.put("x", currentPose.getX());
            packet.put("y", currentPose.getY());
            packet.put("heading", currentPose.getHeading());

            fieldOverlay.setStroke("green");
            DashboardUtil.drawSampledTrajectory(fieldOverlay, mTrajectory);

            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

            mDashboard.sendTelemetryPacket(packet);

            mDrive.update();

            return !mDrive.isFollowingTrajectory();
        }

    }
}
