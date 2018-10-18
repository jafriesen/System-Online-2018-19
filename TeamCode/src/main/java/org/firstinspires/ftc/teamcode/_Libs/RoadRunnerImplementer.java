package org.firstinspires.ftc.teamcode._Libs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.trajectory.DashboardUtil;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode._Robot.RobotDrive;

import java.util.ArrayList;

public class RoadRunnerImplementer {
    static public class FollowTrajectory extends AutoLib.Step {
        TankPIDVAFollower mFollower;
        Trajectory mTrajectory;
        RobotDrive mDrive;
        FtcDashboard mDashboard;

        public FollowTrajectory(HardwareMap hardwareMap, FtcDashboard dashboard) {
            mDrive = new RobotDrive(hardwareMap);
            // change these constraints to something reasonable for your drive
            DriveConstraints baseConstraints = new DriveConstraints(20.0, 30.0, Math.PI / 2, Math.PI / 2);
            TankConstraints constraints = new TankConstraints(baseConstraints, mDrive.getTrackWidth());
            mTrajectory = new TrajectoryBuilder(new Pose2d(0, 0, 0), constraints)
                    .splineTo(new Pose2d(40, 40, 0))
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
                mFollower.followTrajectory(mTrajectory);
            }

            Pose2d currentPose = mDrive.getPoseEstimate();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            fieldOverlay.setStroke("green");
            DashboardUtil.drawSampledTrajectory(fieldOverlay, mTrajectory);
            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);
            mDashboard.sendTelemetryPacket(packet);

            mFollower.update(currentPose);
            mDrive.updatePoseEstimate();

            return !mFollower.isFollowing();
        }

    }
}
