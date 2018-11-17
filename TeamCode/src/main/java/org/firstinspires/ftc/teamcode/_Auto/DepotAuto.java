package org.firstinspires.ftc.teamcode._Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode._Auto.Steps.SampleStep;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.AutoOpMode;
import org.firstinspires.ftc.teamcode._Libs.RoadRunnerImplementer;
import org.firstinspires.ftc.teamcode.drive.SampleTankDriveREVOptimized;

@Autonomous(name="FULL Depot Auto", group="FullAuto")
public class DepotAuto extends AutoOpMode {

    private SampleTankDriveREVOptimized drive;
    private DcMotorEx hangMotor;
    private Servo claimServo;
    private AutoLib.Data data;

    @Override
    public void setup() {
        drive = new SampleTankDriveREVOptimized(hardwareMap);
        hangMotor = mHardwareFactory.getDcMotorEx("lift");
        claimServo = mHardwareFactory.getServo("marker");
        data.Float = 2;

        hangMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        hangMotor.setTargetPosition(0); // hold the hang

        Trajectory leaveHang = drive.trajectoryBuilder()    // drive 5 inches
                .lineTo(new Vector2d(5, 0))
                .build();

        Trajectory sample[] = new Trajectory[] {
                drive.trajectoryBuilder().splineTo(new Pose2d(13, 12, 0)).build(),
                drive.trajectoryBuilder().lineTo(new Vector2d(13, 0)).build(),
                drive.trajectoryBuilder().splineTo(new Pose2d(13, -12, 0)).build()};

        mSequence.add(new AutoLib.RunToPositionMotorStep(hangMotor, 1.0, 0, true)); // unlatch
        mSequence.add(new SampleStep(mVlib, this, data));   // find the cheddar
        mSequence.add(new RoadRunnerImplementer.Follow2dTrajectory(drive, mDashboard, leaveHang));  // move away from lander
        mSequence.add(new AutoLib.RunToPositionMotorStep(hangMotor, 1.0, -5000, true)); // move latch down
        mSequence.add(new RoadRunnerImplementer.Follow2dTrajectory(drive, mDashboard, sample, data)); // hit the cheddar
        mSequence.add(new AutoLib.ServoStep(claimServo, 1.0));  // drop the marker
    }
}