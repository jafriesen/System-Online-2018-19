package org.firstinspires.ftc.teamcode._Auto;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode._Auto.Steps.ChoosePathStep;
import org.firstinspires.ftc.teamcode._Auto.Steps.DoStepsStep;
import org.firstinspires.ftc.teamcode._Auto.Steps.SampleStep;
import org.firstinspires.ftc.teamcode._Auto.Steps.UnlatchStep;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.AutoOpMode;
import org.firstinspires.ftc.teamcode._Libs.HeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.RoadRunnerImplementer;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode.drive.SampleTankDriveREV;
import org.firstinspires.ftc.teamcode.drive.SampleTankDriveREVOptimized;
@Config
@Autonomous(name="FULL Crater Auto", group="FullAuto")
public class CraterAuto extends AutoOpMode {

    private SampleTankDriveREV drive;
    private DcMotor hangMotors;
    private DcMotorEx motors[];
    private DcMotor extendMotor, spinMotor;
    private Servo intake1, intake2;
    private AutoLib.Data data;
    private BNO055IMU gyro;
    private AutoLib.Step samplePath1[], samplePath2[], samplePath3[];
    private DoStepsStep doStepStep;
    private SensorLib.PID pid;

    // parameters of the PID controller for this sequence
    public static double Kp = 0.024;       // motor power proportional term correction per degree of deviation
    public static double Ki = 0.001;         // ... integrator term
    public static double Kd = 0.002;             // ... derivative term
    public static double KiCutoff = 100.0;    // maximum angle error for which we update integrator

    public static double Distance1 = 5;
    public static double Distance2 = 15;
    public static double Angle1 = 50;
    public static double Distance3 = 5;
    public static double Angle2 = 75;
    public static double Distance4 = 35;
    public static double Angle3 = 135;
    public static double Distance5 = 30;
    public static double Distance6 = 20;
    public static double MaxPower = 0.5;
    public static double AngleTolerance = 5.0;

    public CraterAuto() {
        msStuckDetectInit = 10000;
        msStuckDetectInitLoop = 10000;
        msStuckDetectStop = 10000;
    }

    @Override
    public void setup() {
        drive = new SampleTankDriveREV(hardwareMap);

        hangMotors = mHardwareFactory.getDcMotor("l1");

        extendMotor = mHardwareFactory.getDcMotor("extend");
        spinMotor = mHardwareFactory.getDcMotor("spin");

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro.initialize(parameters);

        intake1 = hardwareMap.servo.get("in1");
        intake2 = hardwareMap.servo.get("in2");
        data = new AutoLib.Data();
        data.Float = 2;

        hangMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors = new DcMotorEx[4];

        motors[0] = mHardwareFactory.getDcMotorEx("fr");
        motors[1] = mHardwareFactory.getDcMotorEx("br");
        motors[2] = mHardwareFactory.getDcMotorEx("fl");
        motors[3] = mHardwareFactory.getDcMotorEx("bl");

        motors[0].setDirection(DcMotorEx.Direction.REVERSE);
        motors[1].setDirection(DcMotorEx.Direction.REVERSE);

        for(DcMotorEx em : motors) {
            em.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            em.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        pid = new SensorLib.PID((float) Kp, (float) Ki, (float) Kd, (float) KiCutoff);

        samplePath1 = new AutoLib.Step[3];
        samplePath1[0] = new AutoLib.GyroRotateStep(this, motors, (float) MaxPower, gyro, pid, (float) Angle1, (float) AngleTolerance);
        samplePath1[1] = new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().forward(Distance2).build());
        samplePath1[2] = new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().back(Distance2).build());

        samplePath2 = new AutoLib.Step[2];
        samplePath2[0] = new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().forward(Distance3).build());
        samplePath2[1] = new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().back(Distance3).build());

        samplePath3 = new AutoLib.Step[3];
        samplePath3[0] = new AutoLib.GyroRotateStep(this, motors, (float) MaxPower, gyro, pid, (float) -Angle1, (float) AngleTolerance);
        samplePath3[1] = new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().forward(Distance2).build());
        samplePath3[2] = new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().back(Distance2).build());

        doStepStep = new DoStepsStep(samplePath2);


        mSequence.add(new AutoLib.ServoStep(intake1, 0.0));
        mSequence.add(new AutoLib.ServoStep(intake2, 1.0));
        mSequence.add(new AutoLib.TimedMotorStep(hangMotors, -1.0, 5.0, true));
        mSequence.add(new SampleStep(mVlib, this, data));
        mSequence.add(new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().forward(5).build()));
        mSequence.add(new ChoosePathStep(doStepStep, samplePath1, samplePath2, samplePath3, data));
        mSequence.add(doStepStep);
        mSequence.add(new AutoLib.GyroRotateStep(this, motors, (float) MaxPower, gyro, pid, (float) Angle2, (float) AngleTolerance));
        mSequence.add(new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().forward(Distance4).build()));
        mSequence.add(new AutoLib.GyroRotateStep(this, motors, (float) MaxPower, gyro, pid, (float) Angle3, (float) AngleTolerance));
        mSequence.add(new AutoLib.LogTimeStep(this, "wait", 1.0));
        mSequence.add(new AutoLib.TimedMotorStep(extendMotor, 1.0, 2.0, true));
        mSequence.add(new AutoLib.ServoStep(intake1, 0.5));
        mSequence.add(new AutoLib.ServoStep(intake2, 0.5));
        mSequence.add(new AutoLib.LogTimeStep(this, "wait", 0.5));
        mSequence.add(new AutoLib.TimedMotorStep(spinMotor, 1.0, 1.0, true));
        mSequence.add(new AutoLib.ServoStep(intake1, 0.0));
        mSequence.add(new AutoLib.ServoStep(intake2, 1.0));
        mSequence.add(new AutoLib.TimedMotorStep(extendMotor, -1.0, 2.0, true));
        mSequence.add(new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().back(Distance6).build()));
    }
}