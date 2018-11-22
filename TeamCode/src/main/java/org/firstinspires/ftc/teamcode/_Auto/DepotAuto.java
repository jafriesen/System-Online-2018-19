package org.firstinspires.ftc.teamcode._Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode._Auto.Steps.SampleStep;
import org.firstinspires.ftc.teamcode._Auto.Steps.UnlatchStep;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.AutoOpMode;
import org.firstinspires.ftc.teamcode._Libs.HeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.RoadRunnerImplementer;
import org.firstinspires.ftc.teamcode.drive.SampleTankDriveREV;
import org.firstinspires.ftc.teamcode.drive.SampleTankDriveREVOptimized;

@Autonomous(name="FULL Depot Auto", group="FullAuto")
public class DepotAuto extends AutoOpMode {

    private SampleTankDriveREV drive;
    private DcMotor hangMotors[];
    private DcMotorEx motors[];
    private Servo claimServo;
    private AutoLib.Data data;

    @Override
    public void setup() {
        //drive = new SampleTankDriveREV(hardwareMap);

        hangMotors = new DcMotor[2];
        hangMotors[0] = mHardwareFactory.getDcMotor("l1");
        hangMotors[1] = mHardwareFactory.getDcMotor("l2");

        claimServo = mHardwareFactory.getServo("marker");
        data = new AutoLib.Data();
        data.Float = 2;

        hangMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hangMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangMotors[1].setDirection(DcMotor.Direction.REVERSE);

        motors = new DcMotorEx[4];

        motors[0] = mHardwareFactory.getDcMotorEx("fr");
        motors[1] = mHardwareFactory.getDcMotorEx("br");
        motors[2] = mHardwareFactory.getDcMotorEx("fl");
        motors[3] = mHardwareFactory.getDcMotorEx("bl");

        motors[0].setDirection(DcMotorEx.Direction.REVERSE);
        motors[1].setDirection(DcMotorEx.Direction.REVERSE);

        motors[0].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motors[2].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motors[3].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        /*
        Trajectory leaveHang = drive.trajectoryBuilder()    // drive 5 inches
                .lineTo(new Vector2d(5, 0))
                .build();

        Trajectory sample[] = new Trajectory[] {
                drive.trajectoryBuilder().splineTo(new Pose2d(13, 12, 0)).build(),
                drive.trajectoryBuilder().lineTo(new Vector2d(13, 0)).build(),
                drive.trajectoryBuilder().splineTo(new Pose2d(13, -12, 0)).build()};
        */

        AutoLib.TurnByTimeStep turnStep = new AutoLib.TurnByTimeStep(motors[0], motors[1], motors[2], motors[3], 0.0, 0.0, 0.2,true);

        mSequence.add(new UnlatchStep(this, hangMotors[1], hangMotors[0], -1.0, 3000));
        //mSequence.add(new SampleStep(mVlib, this, data, turnStep));   // find the cheddar
        //mSequence.add(new RoadRunnerImplementer.Follow2dTrajectory(drive, mDashboard, leaveHang));  // move away from lander
        mSequence.add(new AutoLib.MoveByTimeStep(hangMotors, 1, 1.5, true)); // lower the lift
        mSequence.add(new AutoLib.MoveByTimeStep(motors, 0.5, 2.5, true));
        //mSequence.add(new RoadRunnerImplementer.Follow2dTrajectory(drive, mDashboard, sample, data)); // hit the cheddar
        mSequence.add(new AutoLib.ServoStep(claimServo, 1.0));  // drop the marker
        mSequence.add(new AutoLib.MoveByTimeStep(motors, -0.4, 0.5, true));
        mSequence.add(new AutoLib.ServoStep(claimServo, 0.0));
    }
}