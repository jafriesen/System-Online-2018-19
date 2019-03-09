package org.firstinspires.ftc.teamcode._Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._Auto.Steps.ChoosePathStep;
import org.firstinspires.ftc.teamcode._Auto.Steps.DoStepsStep;
import org.firstinspires.ftc.teamcode._Auto.Steps.SampleStep;
import org.firstinspires.ftc.teamcode._Auto.Steps.UnlatchStep;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.AutoOpMode;
import org.firstinspires.ftc.teamcode._Libs.RoadRunnerImplementer;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode.drive.SampleTankDriveREV;

@Config
@Autonomous(name="FULL Double Sample Auto", group="FullAuto")
public class DoubleSample extends AutoOpMode {

    private SampleTankDriveREV drive;
    private DcMotor hangMotors[];
    private DcMotorEx motors[];
    private DcMotor extendMotor, spinMotor;
    private Servo claimServo;
    private Servo intakeBar1, intakeBar2;
    private AutoLib.Data data;
    private BNO055IMU gyro;
    private AutoLib.Step samplePath1[], samplePath2[], samplePath3[];
    private DoStepsStep doStepStep;
    private AutoLib.Step dSamplePath1[], dSamplePath2[], dSamplePath3[];
    private DoStepsStep dDoStepStep;
    private SensorLib.PID pid;

    // parameters of the PID controller for this sequence
    public static double Kp = 0.024;       // motor power proportional term correction per degree of deviation
    public static double Ki = 0.001;         // ... integrator term
    public static double Kd = 0.002;             // ... derivative term
    public static double KiCutoff = 100.0;    // maximum angle error for which we update integrator

    public static double Distance1 = 5;
    public static double Distance2 = 18;
    public static double Angle1 = 45;
    public static double Distance3 = 10;
    public static double Angle2 = 75;
    public static double Distance4 = 35;
    public static double Angle3 = 135;
    public static double Distance5 = 35;
    public static double Distance6 = 50;
    public static double MaxPower = 0.5;
    public static double AngleTolerance = 5.0;
    public static double Angle4 = 70;
    public static double Angle5 = 90;
    public static double Angle6 = 110;
    public static double Distance7 = 30;
    public static double Distance8 = 10;

    public DoubleSample() {
        msStuckDetectInit = 10000;
        msStuckDetectInitLoop = 10000;
        msStuckDetectStop = 10000;
    }

    @Override
    public void setup() {
        drive = new SampleTankDriveREV(hardwareMap);

        hangMotors = new DcMotor[2];
        hangMotors[0] = mHardwareFactory.getDcMotor("l1");
        hangMotors[1] = mHardwareFactory.getDcMotor("l2");

        extendMotor = mHardwareFactory.getDcMotor("extend");
        spinMotor = mHardwareFactory.getDcMotor("spin");

        intakeBar1 = hardwareMap.servo.get("b1");
        intakeBar2 = hardwareMap.servo.get("b2");

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro.initialize(parameters);

        claimServo = mHardwareFactory.getServo("flip");
        data = new AutoLib.Data();
        data.Float = 2;

        hangMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hangMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangMotors[1].setDirection(DcMotor.Direction.REVERSE);

        hangMotors[0].setPower(-0.4);
        hangMotors[1].setPower(-0.4);

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


        dSamplePath1 = new AutoLib.Step[3];
        dSamplePath1[0] = new AutoLib.GyroRotateStep(this, motors, (float) MaxPower, gyro, pid, (float) Angle4, (float) AngleTolerance);
        dSamplePath1[1] = new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().back(Distance7).build());
        dSamplePath1[2] = new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().forward(Distance7).build());

        dSamplePath2 = new AutoLib.Step[3];
        dSamplePath2[0] = new AutoLib.GyroRotateStep(this, motors, (float) MaxPower, gyro, pid, (float) Angle5, (float) AngleTolerance);
        dSamplePath2[1] = new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().back(Distance7).build());
        dSamplePath2[2] = new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().forward(Distance7).build());

        dSamplePath3 = new AutoLib.Step[3];
        dSamplePath3[0] = new AutoLib.GyroRotateStep(this, motors, (float) MaxPower, gyro, pid, (float) Angle6, (float) AngleTolerance);
        dSamplePath3[1] = new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().back(Distance7).build());
        dSamplePath3[2] = new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().forward(Distance7).build());

        dDoStepStep = new DoStepsStep(dSamplePath2);

        mSequence.add(new AutoLib.ServoStep(intakeBar1, 1.00));
        mSequence.add(new AutoLib.ServoStep(intakeBar2, 0.00));
        //mSequence.add(new UnlatchStep(this, hangMotors[1], hangMotors[0], 1.0, 1.8f));
        mSequence.add(new SampleStep(mVlib, this, data));
        mSequence.add(new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().forward((float) Distance1).build()));
        mSequence.add(new AutoLib.MoveByTimeStep(hangMotors, -1, 1.4, true)); // lower the lift
        mSequence.add(new ChoosePathStep(doStepStep, samplePath1, samplePath2, samplePath3, data));
        mSequence.add(new ChoosePathStep(dDoStepStep, dSamplePath1, dSamplePath2, dSamplePath3, data));
        mSequence.add(doStepStep);
        mSequence.add(new AutoLib.GyroRotateStep(this, motors, (float) MaxPower, gyro, pid, (float) Angle2, (float) AngleTolerance));
        mSequence.add(new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().forward(Distance4).build()));
        mSequence.add(new AutoLib.GyroRotateStep(this, motors, (float) MaxPower, gyro, pid, (float) Angle3, (float) AngleTolerance));
        mSequence.add(new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().forward(Distance5).build()));

        mSequence.add(dDoStepStep);
        mSequence.add(new AutoLib.GyroRotateStep(this, motors, (float) MaxPower, gyro, pid, (float) Angle3, (float) AngleTolerance));
        mSequence.add(new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().back(Distance8).build()));

        mSequence.add(new AutoLib.TimedMotorStep(extendMotor, 0.5, 1.5, false));
        mSequence.add(new AutoLib.ServoStep(claimServo, 1.00));
        mSequence.add(new AutoLib.LogTimeStep(this, "wait", 0.5));
        mSequence.add(new AutoLib.LogTimeStep(this, "wait", 0.5));
        mSequence.add(new AutoLib.ServoStep(claimServo, 0.4));
        mSequence.add(new AutoLib.TimedMotorStep(extendMotor, -0.5, 1.5, true));

        mSequence.add(new RoadRunnerImplementer.Follow2dTrajectory(this, drive, drive.trajectoryBuilder().back(Distance6).build()));
    }
}