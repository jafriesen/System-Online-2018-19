package org.firstinspires.ftc.teamcode._Libs;

import android.support.annotation.ColorInt;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

/**
 * Created by phanau on 12/14/15.
 */

// a library of classes that support autonomous opmode programming
public class AutoLib {

    // the base class from which everything else derives.
    // each action in an autonomous sequence is a Step of some kind.
    // a Step may be simple (like run a Motor) or a composite of several Steps which
    // are either run sequentially or in parallel (see Sequences below).
    static public abstract class Step {

        int mLoopCount;     // keeps count of how many times loop() has been called on this Step

        protected Step() {
            mLoopCount = 0;
        }

        // returns true iff called from the first call to loop() on this Step
        public boolean firstLoopCall() {
            return (mLoopCount == 1);    // assume this is called AFTER super.loop()
        }

        // run the next time-slice of the Step; return true when the Step is completed
        public boolean loop() {
            mLoopCount++;       // increment the loop counter
            return false;
        }

        // reset the Step so it can be used repeatedly by teleop modes, too
        public void reset() {
            mLoopCount = 0;
        }

        // get count of how many times loop() has been called on this step
        public int loopCount() { return mLoopCount; }

    }

    // ------------------ some implementations of Step aggregation constructs -------------------------

    // base class for Sequences that perform multiple Steps, either sequentially or concurrently
    static public abstract class Sequence extends Step {
        protected ArrayList<Step> mSteps;  // expandable array containing the Steps in the Sequence

        protected Sequence() {
            mSteps = new ArrayList<Step>(10);   // create the array with an initial capacity of 10
        }

        // add a Step to the end of the Sequence
        public Step add(Step step) {
            mSteps.add(step);
            return this;        // allows daisy-chaining of calls
        }

        // add a Step to the beginning of the Sequence - used for a control steps that
        // needs to run BEFORE the e.g. motor steps it controls
        public Step preAdd(Step step) {
            mSteps.add(0, step);
            return this;
        }

        // run the next time-slice of the Sequence; return true when the Sequence is completed.
        public boolean loop() {
            super.loop();
            return false;
        }

        // reset all the Steps in the Sequence
        public void reset() {
            super.reset();
            for (Step s : mSteps)
                s.reset();
        }
    }

    // a Sequence that performs its constituent Steps sequentially
    static public class LinearSequence extends Sequence {
        int mIndex;     // index of currently active Step

        public LinearSequence() {
            mIndex = 0;     // start at the beginning
        }

        // run the current Step of the Sequence until it completes, then the next Step and
        // the next, etc., etc. until the last Step completes, at which point the Sequence
        // returns complete.
        public boolean loop() {
            super.loop();
            if (mIndex < mSteps.size()) {       // if this Sequence is not completed ...
                if (mSteps.get(mIndex).loop())  // if this Step is complete, move to the next Step
                    mIndex++;
            }
            return (mIndex >= mSteps.size());   // return true when last Step completes
        }

        // reset this Sequence to its beginning
        public void reset() {
            super.reset();
            mIndex = 0;
        }
    }

    // a Sequence that performs its constituent Steps concurrently
    static public class ConcurrentSequence extends Sequence {

        public ConcurrentSequence() {
        }

        // run all the Steps in the Sequence "concurrently" -- i.e. run the loop() function of
        // each of the Steps each time loop() is called. When ALL the Steps report that they
        // are done, then this Sequence is done.
        public boolean loop() {
            super.loop();
            boolean bDone = true;
            for (Step s : mSteps)
                bDone &= s.loop();      // "done" means ALL Steps are done
            return bDone;
        }

        // reset: let superclass take care of it
    }

    // a Step that implements an if-then construct --
    // if the "if" step returns true, execute the "then" step, else execute the "else" step.
    // note that the evaluation of the "if" clause happens EVERY TIME through the loop(), not just once,
    // so if you want only the first evaluation to control the behavior of future loop() calls
    // you need to "latch" the result in your "if" step.
    static public class IfThenStep extends Step {
        Step mIf;
        Step mThen;
        Step mElse;

        public IfThenStep(Step ifStep,  Step thenStep, Step elseStep) {
            mIf = ifStep;
            mThen = thenStep;
            mElse = elseStep;
        }

        public boolean loop() {
            if (mIf.loop())
                return mThen.loop();
            else
                return mElse.loop();
        }

        public void reset() {
            super.reset();
            mIf.reset();
            mThen.reset();
            mElse.reset();
        }
    }

    // a Step that implements a do-until construct --
    // repeat the "do" Step until the "until" Step returns done.
    // reset the "do" Step each time "until" Step says do it again.
    static public class DoUntilStep extends Step {
        Step mDo;
        Step mUntil;

        public DoUntilStep(Step doStep, Step untilStep) {
            mDo = doStep;
            mUntil = untilStep;
        }

        public boolean loop() {
            boolean done = false;
            if (mDo.loop()) {             // if do content is done ...
                if (mUntil.loop()) {      // and until test is done ...
                    done = true;          // we're done
                }
                else {
                    mDo.reset();          // reset the do content so we can do it again ...
                }
            }
            return done;
        }

        public void reset() {
            super.reset();
            mDo.reset();
            mUntil.reset();
        }
    }

    // a Step that waits until a given test Step has returned true for a given length of time.
    // e.g. if you want to turn to a given heading, you might wait until a "read gyro" step (like GyroTestHeadingStep)
    // has reported that you're within 3 degrees of a given heading for 1 second to make
    // sure you've actually settled at that heading and aren't just "passing through it".
    static public class WaitTimeTestStep extends Step {
        Step mTest;         // the "test" Step
        Timer mTimer, mTimer2;       // Timer for this Step

        public WaitTimeTestStep(Step test, double seconds, double seconds2) {
            mTest = test;
            mTimer = new Timer(seconds);
            mTimer2 = new Timer(seconds2);
        }

        public boolean loop() {
            super.loop();

            // start the Timer on our first call
            if (firstLoopCall()) {
                mTimer.start();
                mTimer2.start();
            }

            // if the test Step fails, restart the Timer; else let it keep running
            if (!mTest.loop()) {
                mTimer.start();
                mTimer2.start();
            }

            // return true when time is exhausted --
            // i.e. when the Test has succeeded continuously for the given time
            if(mTimer.done()) {
                return true;
            }
            if(mTimer2.done()) {
                return true;
            }
            else {
                return false;
            }
        }
    }

    // ------------------ some implementations of primitive Steps ----------------------------

    // a simple Step that just logs its existence for a given number of loop() calls
    // really just for testing sequencer stuff, not for actual use on a robot.
    static public class LogCountStep extends Step {
        AutoOpMode mOpMode;     // needed so we can log output
        String mName;       // name of the output field
        int mCount;         // current loop count of this Step
        int mCount0;     // original loop count

        public LogCountStep(AutoOpMode opMode, String name, int loopCount) {
            mOpMode = opMode;
            mName = name;
            mCount = mCount0 = loopCount;
        }

        public boolean loop() {
            super.loop();

            // log some info about this Step
            if (mCount > 0) {
                mOpMode.packet.put(mName, "count = " + mCount);
                mCount--;
            } else
                mOpMode.packet.put(mName, "done");


            // return true when count is exhausted
            return (mCount <= 0);
        }

        // reset the Step so it can be used repeatedly by teleop modes, too
        public void reset() {
            super.reset();
            mCount = mCount0;
        }

    }

    // a simple Step that just logs its existence for a given length of time
    static public class LogTimeStep extends Step {
        AutoOpMode mOpMode;     // needed so we can log output
        String mName;       // name of the output field
        Timer mTimer;       // Timer for this Step

        public LogTimeStep(AutoOpMode opMode, String name, double seconds) {
            mOpMode = opMode;
            mName = name;
            mTimer = new Timer(seconds);
        }

        public boolean loop() {
            super.loop();

            // start the Timer on our first call
            if (firstLoopCall())
                mTimer.start();

            // log some info about this Step
            if (!mTimer.done())        // appears to cycle here at about 3ms/loop
                mOpMode.packet.put(mName, "time = " + mTimer.remaining());
            else
                mOpMode.packet.put(mName, "done");

            // return true when time is exhausted
            return (mTimer.done());
        }

    }

    // interface for setting the current power of any kind of MotorStep
    public interface SetPower {
        public void setPower(double power);
    }

    // a Step that runs a DcMotor at a given power, for a given time
    static public class TimedMotorStep extends Step implements SetPower {
        Timer mTimer;
        DcMotor mMotor;
        double mPower;
        boolean mStop;          // stop motor when count is reached

        public TimedMotorStep(DcMotor motor, double power, double seconds, boolean stop) {
            mMotor = motor;
            mPower = power;
            mTimer = new Timer(seconds);
            mStop = stop;
        }

        // for dynamic adjustment of power during the Step
        public void setPower(double power) {
            mPower = power;
        }

        public void setTime(double time) {
            mTimer = new Timer(time);
        }

        public boolean loop() {
            super.loop();

            // start the Timer and start the motor on our first call
            if (firstLoopCall()) {
                mTimer.start();
                mMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(mTimer.remaining() > 0.00001) {
                    mMotor.setPower(mPower);
                }
                else
                    return true;
            }

            // run the motor at the requested power until the Timer runs out
            boolean done = mTimer.done();
            if (done && mStop)
                mMotor.setPower(0);
            else
                mMotor.setPower(mPower);        // update power in case it changed

            return done;
        }

    }

    // a Step that runs a DcMotor at a given power, for a given encoder count
    static public class RunToPositionMotorStep extends Step implements SetPower {
        DcMotor mMotor;    // motor to control
        double mPower;          // power level to use
        int mEncoderCount;      // target encoder count
        boolean mStop;          // stop motor when count is reached

        public RunToPositionMotorStep(DcMotor motor, double power, int count, boolean stop) {
            mMotor = motor;
            mPower = power;
            mEncoderCount = count;
            mStop = stop;
        }

        // for dynamic adjustment of power during the Step
        public void setPower(double power) {
            mPower = power;
        }

        public void setEncoderCount(int position) {
            mEncoderCount = position;
        }

        public boolean loop() {
            super.loop();

            boolean done = false;

            // we need a little state machine to make the encoders happy
            if (firstLoopCall()) {
                // set up the motor on our first call
                mMotor.setTargetPosition(mMotor.getCurrentPosition() + mEncoderCount);
                mMotor.setPower(mPower);
            }

            // the rest of the time, just update power and check to see if we're done
            done = !mMotor.isBusy();
            if (done && mStop)
                mMotor.setPower(0);     // optionally stop motor when target reached
            else
                mMotor.setPower(mPower);        // update power in case it changed

            return done;
        }

    }

    static public class EncoderMotorStep extends AutoLib.Step implements AutoLib.SetPower {
        private float mPower;
        private float mCounts;
        private OpMode mOpMode;
        private SensorLib.PID mPid;
        private double mPrevTime;
        private DcMotor mMotor;
        private int done = 0;

        public EncoderMotorStep(OpMode mode, DcMotor motor, float counts, SensorLib.PID pid) {
            mOpMode = mode;
            mPid = pid;
            mMotor = motor;
            mCounts = counts;
        }

        public boolean loop()
        {
            super.loop();

            if (firstLoopCall()) {
                mPrevTime = mOpMode.getRuntime();
                mCounts = mCounts + mMotor.getCurrentPosition();
                mMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            float error = mCounts - mMotor.getCurrentPosition();

            double time = mOpMode.getRuntime();
            double dt = time - mPrevTime;
            mPrevTime = time;

            float correction = mPid.loop(error, (float)dt);

            mMotor.setPower(correction);

            return false;
        }

        @Override
        public void setPower(double power) {
            mPower = (float) power;
        }
    }

    // a Step that drives a Servo to a given position
    // it would be nice if we got actual position info back from the servo, but that's not
    // how it works, so we just wait long enough for it to probably get where it's told to go.
    static public class ServoStep extends Step {
        Servo mServo;
        double mPosition;          // target position of servo
        Timer mTimer;              // Timer for this Step

        public ServoStep(Servo servo, double position) {
            mServo = servo;
            mPosition = position;
            mTimer = null;
        }

        public boolean loop() {
            super.loop();

            if (firstLoopCall()) {
                // tell the servo to go to the target position on the first call
                mServo.setPosition(mPosition);

                // and start a timer that estimates when the motion will complete
                // assuming servo goes at about 300 degrees/sec and 0..1 range is about 180 degrees
                double seconds = (mPosition-mServo.getPosition());
                mTimer = new Timer(seconds);
                mTimer.start();
            }

            // we're done when we've waited long enough for
            // the servo to (probably) get to the ordered position
            boolean done = mTimer.done();

            return done;
        }

    }

    static public class Data {
        public float Float;
        public Data() {
            Float = 0;
        }
    }

    // some utility functions

    // linear interpolation
    private static double lerp (double x, double x0, double x1, double y0, double y1)
    {
        return ((x-x0)/(x1-x0))*(y1-y0) + y0;
    }

    // return normalization factor that makes max magnitude of any argument 1
    static public float normalize(float a, float b)
    {
        float m = Math.max(Math.abs(a), Math.abs(b));
        return (m > 1.0f) ? 1.0f/m : 1.0f;
    }

    static public float normalize(float f, float a, float b)
    {
        float m = Math.max(Math.abs(a), Math.abs(b));
        return (m > f) ? f/m : 1.0f;
    }

    static public float normalize(float a, float b, float c, float d)
    {
        float m = Math.max(Math.max(Math.abs(a), Math.abs(b)), Math.max(Math.abs(c), Math.abs(d)));
        return (m > 1.0f) ? 1.0f/m : 1.0f;
    }

    // return normalization factor that makes max magnitude of any argument 1
    static public double normalize(double a, double b)
    {
        double m = Math.max(Math.abs(a), Math.abs(b));
        return (m > 1.0) ? 1.0/m : 1.0;
    }

    static public double normalize(double a, double b, double c, double d)
    {
        double m = Math.max(Math.max(Math.abs(a), Math.abs(b)), Math.max(Math.abs(c), Math.abs(d)));
        return (m > 1.0) ? 1.0/m : 1.0;
    }

    static public double normalize(double f, double a, double b, double c, double d)
    {
        double m = Math.max(Math.max(Math.abs(a), Math.abs(b)), Math.max(Math.abs(c), Math.abs(d)));
        return (m > f) ? f/m : 1.0;
    }

    static public float normalize(float[] a)
    {
        float m = 0;
        for (float x : a)
            m = Math.max(m, Math.abs(x));
        return (m > 1.0f) ? 1.0f/m : 1.0f;
    }

    static public double normalize(double[] a)
    {
        double m = 0;
        for (double x : a)
            m = Math.max(m, Math.abs(x));
        return (m > 1.0) ? 1.0/m : 1.0;
    }

    // interface for setting the current power of either kind of MotorStep
    public interface SetMotorSteps {
        public void set(ArrayList<AutoLib.SetPower> motorsteps);
    }

    static public abstract class MotorGuideStep extends AutoLib.Step implements SetMotorSteps {
        public void set(ArrayList<AutoLib.SetPower> motorsteps){}
    }

    // some Steps that use various sensor input to control other Steps

    // a Step that provides gyro-based guidance to motors controlled by other concurrent Steps (e.g. encoder or time-based)
    // assumes an even number of concurrent drive motor steps in order right ..., left ...
    // this step tries to keep the robot on the given course by adjusting the left vs. right motors to change the robot's heading.
    static public class GyroGuideStep extends MotorGuideStep {
        private float mPower;                               // basic power setting of all 4 motors -- adjusted for steering along path
        private float mMaxPower = 1.0f;                     // maximum power for any motor
        private float mHeading;                             // compass heading to steer for (-180 .. +180 degrees)
        private AutoOpMode mOpMode;                             // needed so we can log output (may be null)
        private BNO055IMU mGyro;                        // sensor to use for heading information (e.g. Gyro or Vuforia)
        private SensorLib.PID mPid;                         // proportional–integral–derivative controller (PID controller)
        private double mPrevTime;                           // time of previous loop() call
        private ArrayList<SetPower> mMotorSteps;            // the motor steps we're guiding - assumed order is right ... left ...

        public GyroGuideStep(AutoOpMode mode, float heading, BNO055IMU gyro, SensorLib.PID pid,
                             ArrayList<SetPower> motorsteps, float power) {
            mOpMode = mode;
            mHeading = heading;
            mGyro = gyro;
            mPid = pid;
            mMotorSteps = motorsteps;
            mPower = power;
        }

        public void setMaxPower(float maxPower) {
            mMaxPower = maxPower;
        }

        public boolean loop()
        {
            super.loop();

            // initialize previous-time on our first call -> dt will be zero on first call
            if (firstLoopCall()) {
                mPrevTime = mOpMode.getRuntime();           // use timer provided by AutoOpMode
            }

            float heading = mGyro.getAngularOrientation().firstAngle;     // get latest reading from direction sensor
            // convention is positive angles CCW, wrapping from 359-0

            float error = SensorLib.Utils.wrapAngle(heading-mHeading);   // deviation from desired heading
            // deviations to left are positive, to right are negative

            // compute delta time since last call -- used for integration time of PID step
            double time = mOpMode.getRuntime();
            double dt = time - mPrevTime;
            mPrevTime = time;

            // feed error through PID to get motor power correction value
            float correction = -mPid.loop(error, (float)dt);

            // compute new right/left motor powers
            float rightPower = mPower + correction;
            float leftPower = mPower - correction;

            // normalize so neither has magnitude > 1
            float norm = normalize(mMaxPower, rightPower, leftPower);
            rightPower *= norm;
            leftPower *= norm;

            // set the motor powers -- handle both time-based and encoder-based motor Steps
            // assumed order is right motors followed by an equal number of left motors
            int i = 0;
            for (SetPower ms : mMotorSteps) {
                ms.setPower((i++ < mMotorSteps.size()/2) ? rightPower : leftPower);
            }

            // log some data
            if (mOpMode != null) {
                mOpMode.packet.put("heading ", heading);
                mOpMode.packet.put("left power ", leftPower);
                mOpMode.packet.put("right power ", rightPower);

                mOpMode.telemetry.addData("heading", heading);
            }

            // guidance step always returns "done" so the CS in which it is embedded completes when
            // all the motors it's controlling are done
            return true;
        }

        // set motor control steps this step should control (assumes ctor called with null argument)
        public void set(ArrayList<AutoLib.SetPower> motorsteps)
        {
            mMotorSteps = motorsteps;
        }

        // update target direction, heading, and power --
        // used by interactive teleop modes to redirect the step from controller input
        public void set(float heading,float power) {
            mHeading = heading;
            mPower = power;
        }

    }

    // a Step that waits for valid location and heading data to be available --- e.g from Vuforia --
    // when added to either a dead reckoning or gyro-based movement step, it can be used to end that step
    // when we're close enough to the targets for Vuforia to start being used. The base step should be
    // of "zero-length" -- i.e. it should always be "done" so the composite step will be "done" as soon as
    // this step detects valid location and heading data.
    static public class LocationHeadingWaitStep extends Step {

        private LocationSensor mLocSensor;                  // sensor to use for field location information (e.g. Vuforia)
        private HeadingSensor mYawSensor;                   // sensor to use for robot orientation on field (may be same as LocationSensor)

        public LocationHeadingWaitStep(LocationSensor locSensor, HeadingSensor yawSensor)
        {
            mLocSensor = locSensor;
            mYawSensor = yawSensor;
        }

        public boolean loop()
        {
            super.loop();

            boolean bDone = true;
            if (mLocSensor != null)
                bDone &= mLocSensor.haveLocation();
            if (mYawSensor != null)
                bDone &= mYawSensor.haveHeading();
            return bDone;
        }
    }

    // a Step that returns true if the given HeadingSensor reports a heading within some tolerance of a desired heading.
    static public class GyroTestHeadingStep extends Step {
        private BNO055IMU mSensor;
        private double mHeading;
        private double mTolerance;
        private Timer mTimer;

        public GyroTestHeadingStep(BNO055IMU sensor, double heading, double tol){
            mSensor = sensor;
            mHeading = heading;
            mTolerance = tol;
            mTimer = new Timer(3);
        }

        public boolean loop() {
            super.loop();

            if(firstLoopCall()) {
                mTimer.start();
            }
            if(mTimer.done()) {
                return true;
            }
            return (Math.abs(mSensor.getAngularOrientation().firstAngle-mHeading) < mTolerance);
        }

        void set(float heading) {
            mHeading = heading;
        }
    }

    // a Step that stops the given set of motor control steps and terminates
    // when the given DistanceSensor reports less than a given distance (in mm).
    // pass in an empty set of motors to just do the test (e.g. for use with WaitTimeTestStep).
    static public class DistanceSensorGuideStep extends Step {

        private AutoOpMode mOpMode;                     // for telemetry output (optional)
        private DistanceSensor mDistanceSensor;     // distance sensor to read
        private float mDistance;                    // stopping distance
        private ArrayList<SetPower> mSteps;         // motors to stop at given distance

        public DistanceSensorGuideStep(AutoOpMode opMode, DistanceSensor ds, float distance, ArrayList<SetPower> steps)
        {
            mOpMode = opMode;
            mDistanceSensor = ds;
            mDistance = distance;
            mSteps = steps;
        }

        public boolean loop()
        {
            super.loop();

            boolean have = mDistanceSensor.haveDistance();
            float dist = mDistanceSensor.getDistance();
            boolean done = have && (dist < mDistance);
            if (done)
                for (SetPower step : mSteps)
                    step.setPower(0);
            if (mOpMode != null) {
                //mOpMode.packet.put("DSGS: ", "have = %b  dist(mm) = %2.1f  (in) = %2.1f  done = %b", have, dist, dist/25.4f, done);
            }
            return done;
        }
    }


    // a generic Step that uses a MotorGuideStep to steer the robot while driving along a given path
    // until the terminatorStep tells us that we're there, thereby terminating this step.
    static public class GuidedTerminatedDriveStep extends AutoLib.ConcurrentSequence {

        public GuidedTerminatedDriveStep(AutoOpMode mode, AutoLib.MotorGuideStep guideStep, AutoLib.Step terminatorStep, DcMotor[] motors)
        {
            // add a concurrent Step to control each motor
            ArrayList<AutoLib.SetPower> steps = new ArrayList<AutoLib.SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    AutoLib.TimedMotorStep step = new AutoLib.TimedMotorStep(em, 0, 0, false);
                    // the terminatorStep will stop the motors and complete the sequence
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step that terminates the whole sequence when we're "there"
            if(terminatorStep != null) {
                this.preAdd(terminatorStep);
            }

            // tell the guideStep about the motor Steps it should control
            guideStep.set(steps);

            // add a concurrent Step to control the motor steps based on gyro input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            // and BEFORE the terminatorStep might try to turn the motors off.
            this.preAdd(guideStep);
        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

    }

    static public class GyroRotateStep extends ConcurrentSequence {

        AutoLib.WaitTimeTestStep terminateStep;
        AutoLib.GyroGuideStep guideStep;
        GyroTestHeadingStep testStep;

        public GyroRotateStep(AutoOpMode mode, DcMotor[] motors, float maxPower, BNO055IMU gyro, SensorLib.PID pid, float heading, float tolerance) {
            // add a concurrent Step to control each motor
            ArrayList<AutoLib.SetPower> steps = new ArrayList<AutoLib.SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    AutoLib.TimedMotorStep step = new AutoLib.TimedMotorStep(em, 0, 0, false);
                    // the terminatorStep will stop the motors and complete the sequence
                    this.add(step);
                    steps.add(step);
                }

            testStep = new GyroTestHeadingStep(gyro, heading, tolerance);
            terminateStep = new AutoLib.WaitTimeTestStep(testStep, 2, 3);
            guideStep = new AutoLib.GyroGuideStep(mode, heading, gyro, pid, null, 0);
            guideStep.setMaxPower(maxPower);

            // tell the guideStep about the motor Steps it should control
            guideStep.set(steps);

            this.preAdd(terminateStep);
            this.preAdd(guideStep);
        }

        public void set(float heading) {
            guideStep.set(heading, 0);
            testStep.set(heading);
        }
    }

    // a Step that provides gyro-based guidance to motors controlled by other concurrent Steps (e.g. encoder or time-based)
    // driving "squirrely wheels" that can move sideways by differential turning of front vs. back wheels.
    // assumes 4 concurrent drive motor steps in order right front, right back, left front, left back.
    // this step tries to maintain the robot's absolute orientation (heading) given by the gyro by adjusting the left vs. right motors
    // while the front vs. back power is adjusted to translate in the desired absolute direction.
    static public class SquirrelyGyroGuideStep extends AutoLib.MotorGuideStep {
        private float mPower;                               // basic power setting of all 4 motors -- adjusted for steering along path
        private float mDirection;                           // relative direction along which the robot should move (0 ahead; positive CCW)
        private float mHeading;                             // orientation the robot should maintain while moving
        private float mMaxPower;
        private AutoOpMode mOpMode;                             // needed so we can log output (may be null)
        private HeadingSensor mGyro;                        // sensor to use for heading information (e.g. Gyro or Vuforia)
        private SensorLib.PID mPid;                         // proportional–integral–derivative controller (PID controller)
        private double mPrevTime;                           // time of previous loop() call
        private ArrayList<AutoLib.SetPower> mMotorSteps;    // the motor steps we're guiding - assumed order is fr, br, fl, bl

        public SquirrelyGyroGuideStep(AutoOpMode mode, float direction, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                      ArrayList<AutoLib.SetPower> motorsteps, float power) {
            mOpMode = mode;
            mDirection = direction;
            mHeading = heading;
            mGyro = gyro;
            if (pid != null)
                mPid = pid;     // client is supplying PID controller for correcting heading errors
            else {
                // construct a default PID controller for correcting heading errors
                final float Kp = 0.05f;        // degree heading proportional term correction per degree of deviation
                final float Ki = 0.02f;        // ... integrator term
                final float Kd = 0.0f;         // ... derivative term
                final float KiCutoff = 3.0f;   // maximum angle error for which we update integrator
                mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);
            }
            mMotorSteps = motorsteps;
            mPower = power;
            mMaxPower = 1.0f;
        }

        public void setMaxPower(float maxPower) {
            mMaxPower = maxPower;
        }

        // set motor control steps this step should control (assumes ctor called with null argument)
        public void set(ArrayList<AutoLib.SetPower> motorsteps)
        {
            mMotorSteps = motorsteps;
        }

        // update target direction, heading, and power --
        // used by interactive teleop modes to redirect the step from controller input
        public void set(float direction, float heading, float power)
        {
            mDirection = direction;
            mHeading = heading;
            mPower = power;
        }

        public boolean loop()
        {
            super.loop();

            // initialize previous-time on our first call -> dt will be zero on first call
            if (firstLoopCall()) {
                mPrevTime = mOpMode.getRuntime();           // use timer provided by AutoOpMode
            }

            final float heading = mGyro.getHeading();     // get latest reading from direction sensor
            // convention is positive angles CCW, wrapping from 359-0

            final float error = SensorLib.Utils.wrapAngle(heading - mHeading);   // deviation from desired heading
            // deviations to left are positive, to right are negative

            // compute delta time since last call -- used for integration time of PID step
            final double time = mOpMode.getRuntime();
            final double dt = time - mPrevTime;
            mPrevTime = time;

            // feed error through PID to get motor power value for heading correction
            float hdCorr = mPid.loop(error, (float) dt);

            // relative direction we want to move is difference between given absolute direction and orientation
            float relDir = SensorLib.Utils.wrapAngle(mDirection - heading);

            // calculate relative front/back motor powers for fancy wheels to move us in requested relative direction
            AutoLib.MotorPowers mp = AutoLib.GetSquirrelyWheelMotorPowers(relDir);

            // calculate powers of the 4 motors
            double pFR = mp.Front() * mPower - hdCorr;
            double pBR = mp.Back() * mPower - hdCorr;
            double pFL = mp.Front() * mPower + hdCorr;
            double pBL = mp.Back() * mPower + hdCorr;

            // normalize powers so none has magnitude > 1
            double norm = normalize(mMaxPower, pFR, pBR, pFL, pBL);
            pFR *= norm;  pBR *= norm;  pFL *= norm;  pBL *= norm;

            // set the powers
            mMotorSteps.get(0).setPower(pFR);   // fr
            mMotorSteps.get(1).setPower(pBR);   // br
            mMotorSteps.get(2).setPower(pFL);   // fl
            mMotorSteps.get(3).setPower(pBL);   // bl

            // log some data
            if (mOpMode != null) {
                mOpMode.packet.put("heading ", heading);
                mOpMode.packet.put("front power ", mp.Front());
                mOpMode.packet.put("back power ", mp.Back());
            }

            // guidance step always returns "done" so the CS in which it is embedded completes when
            // all the motors it's controlling are done
            return true;
        }

    }


    // some Steps that combine various motor driving Steps with guide Steps that control them

    // a Step that uses gyro input to drive along a given course for a given time.
    // uses a GyroGuideStep to adjust power to 2 or 4 motors.
    // assumes a robot with up to 4 drive motors in assumed order right motors, left motors
    static public class AzimuthTimedDriveStep extends ConcurrentSequence {

        public AzimuthTimedDriveStep(AutoOpMode mode, float heading, BNO055IMU gyro, SensorLib.PID pid,
                                     DcMotor motors[], float power, float time, boolean stop)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    TimedMotorStep step = new TimedMotorStep(em, power, time, stop);
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on gyro input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            this.preAdd(new GyroGuideStep(mode, heading, gyro, pid, steps, power));

        }

        public AzimuthTimedDriveStep(AutoOpMode mode, float heading, BNO055IMU gyro, SensorLib.PID pid,
                                     DcMotorEx motors[], float power, float time, boolean stop)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    TimedMotorStep step = new TimedMotorStep(em, power, time, stop);
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on gyro input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            this.preAdd(new GyroGuideStep(mode, heading, gyro, pid, steps, power));

        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

        // update target direction, heading, and power --
        // used by interactive teleop modes to redirect the step from controller input
        public void set(float heading,float power) {
            ((GyroGuideStep)mSteps.get(0)).set(heading, power);
        }
    }

    // a Step that uses gyro input to drive along a given course for a given distance given by motor encoders.
    // uses a GyroGuideStep to adjust power to 2 or 4 motors.
    // assumes a robot with up to 4 drive motors in assumed order right motors, left motors
    static public class AzimuthCountedDriveStep extends ConcurrentSequence {

        public AzimuthCountedDriveStep(AutoOpMode mode, float heading, BNO055IMU gyro, SensorLib.PID pid,
                                       DcMotor motors[], float power, int count, boolean stop)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    RunToPositionMotorStep step = new RunToPositionMotorStep(em, power, count, stop);
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on gyro input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            this.preAdd(new GyroGuideStep(mode, heading, gyro, pid, steps, power));

        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

        // update target direction, heading, and power --
        // used by interactive teleop modes to redirect the step from controller input
        public void set(float heading,float power) {
            ((GyroGuideStep)mSteps.get(0)).set(heading, power);
        }
    }

    // a Step that uses gyro input to drive along a given course until the given DistanceSensor
    // reports that we are within a given distance of some target.
    // uses a GyroGuideStep to adjust power to 2 or 4 motors.
    // assumes a robot with up to 4 drive motors in assumed order right motors, left motors
    static public class AzimuthDistanceDriveStep extends ConcurrentSequence {

        public AzimuthDistanceDriveStep(AutoOpMode mode, float heading, BNO055IMU gyro, SensorLib.PID pid,
                                        DcMotor motors[], float power, DistanceSensor ds, float distance)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    TimedMotorStep step = new TimedMotorStep(em, power, 0, false);
                    // the DistanceSensorGuideStep will stop the motors and complete the sequence
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on DistanceSensor input
            // put it 2nd in the list so it can set the power of the motors to zero BEFORE their steps run
            this.preAdd(new DistanceSensorGuideStep(mode, ds, distance, steps));

            // add a concurrent Step to control the motor steps based on gyro input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            // and BEFORE the DistanceSensorGuideStep might try to turn the motors off.
            this.preAdd(new GyroGuideStep(mode, heading, gyro, pid, steps, power));

        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

    }

    // a Step that uses gyro input to stabilize the robot orientation while driving along a given absolute heading
    // using squirrely wheels, for a given time.
    // uses a SquirrelyGyroGuideStep to adjust power to 4 motors in assumed order fr, br, fl, bl
    static public class SquirrelyGyroTimedDriveStep extends AutoLib.ConcurrentSequence {

        public SquirrelyGyroTimedDriveStep(AutoOpMode mode, float direction, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                           DcMotor motors[], float power, float time, boolean stop)
        {
            // add a concurrent Step to control each motor
            ArrayList<AutoLib.SetPower> steps = new ArrayList<AutoLib.SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    TimedMotorStep step = new TimedMotorStep(em, 0, time, stop);
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on gyro input
            this.preAdd(new SquirrelyGyroGuideStep(mode, direction, heading, gyro, pid, steps, power));
        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

        // update target direction, heading, and power --
        // used by interactive teleop modes to redirect the step from controller input
        public void set(float direction, float heading,float power) {
            ((SquirrelyGyroGuideStep)mSteps.get(0)).set(direction, heading, power);
        }
    }

    // a Step that uses gyro input to stabilize the robot orientation while driving along a given absolute heading
    // using squirrely wheels, for a given number of motor counts.
    // uses a SquirrelyGyroGuideStep to adjust power to 4 motors in assumed order fr, br, fl, bl
    static public class SquirrelyGyroCountedDriveStep extends ConcurrentSequence {

        public SquirrelyGyroCountedDriveStep(AutoOpMode mode, float direction, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                             DcMotor motors[], float power, int count, boolean stop)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    RunToPositionMotorStep step = new RunToPositionMotorStep(em, power, count, stop);
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on gyro input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            this.preAdd(new SquirrelyGyroGuideStep(mode, direction, heading, gyro, pid, steps, power));

        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

        // update target direction, heading, and power --
        // used by interactive teleop modes to redirect the step from controller input
        public void set(float direction, float heading,float power) {
            ((SquirrelyGyroGuideStep)mSteps.get(0)).set(direction, heading, power);
        }
    }

    // a Step that uses gyro input to stabilize the robot orientation while driving along a given relative heading
    // using squirrely wheels, until a DistanceSensor reports we're within a given distance of something.
    // uses a SquirrelyGyroGuideStep to adjust power to 4 motors in assumed order fr, br, fl, bl
    // and a DistanceSensorGuideStep to determine when to stop.
    static public class SquirrelyGyroDistanceDriveStep extends ConcurrentSequence {

        public SquirrelyGyroDistanceDriveStep(AutoOpMode mode, float direction, float heading, HeadingSensor gyro, SensorLib.PID pid,
                                              DcMotor motors[], float power, DistanceSensor ds, float distance)
        {
            // add a concurrent Step to control each motor
            ArrayList<SetPower> steps = new ArrayList<SetPower>();
            for (DcMotor em : motors)
                if (em != null) {
                    TimedMotorStep step = new TimedMotorStep(em, power, 0, false);
                    // the DistanceSensorGuideStep will stop the motors and complete the sequence
                    this.add(step);
                    steps.add(step);
                }

            // add a concurrent Step to control the motor steps based on DistanceSensor input
            // put it 2nd in the list so it can set the power of the motors to zero BEFORE their steps run
            this.preAdd(new DistanceSensorGuideStep(mode, ds, distance, steps));

            // add a concurrent Step to control the motor steps based on gyro input
            // put it at the front of the list so it can update the motors BEFORE their steps run
            // and BEFORE the DistanceSensorGuideStep might try to turn the motors off.
            this.preAdd(new SquirrelyGyroGuideStep(mode, direction, heading, gyro, pid, steps, power));

        }

        // the base class loop function does all we need -- it will return "done" when
        // all the motors are done.

    }


    // some convenience utility classes for common operations

    // a Sequence that moves an up-to-four-motor robot in a straight line with given power for given time
    static public class MoveByTimeStep extends ConcurrentSequence {

        public MoveByTimeStep(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double power, double seconds, boolean stop)
        {
            if (fr != null)
                this.add(new TimedMotorStep(fr, power, seconds, stop));
            if (br != null)
                this.add(new TimedMotorStep(br, power, seconds, stop));
            if (fl != null)
                this.add(new TimedMotorStep(fl, power, seconds, stop));
            if (bl != null)
                this.add(new TimedMotorStep(bl, power, seconds, stop));
        }

        public MoveByTimeStep(DcMotor motors[], double power, double seconds, boolean stop)
        {
            for (DcMotor em : motors)
                if (em != null)
                    this.add(new TimedMotorStep(em, power, seconds, stop));
        }

    }


    // a Sequence that turns an up-to-four-motor robot by applying the given right and left powers for given time
    static public class TurnByTimeStep extends ConcurrentSequence {
        TimedMotorStep motorStep[];

        public TurnByTimeStep(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double rightPower, double leftPower, double seconds, boolean stop)
        {
            motorStep = new TimedMotorStep[4];
            if (fr != null)
                this.add(motorStep[0] = new TimedMotorStep(fr, rightPower, seconds, stop));
            if (br != null)
                this.add(motorStep[1] = new TimedMotorStep(br, rightPower, seconds, stop));
            if (fl != null)
                this.add(motorStep[2] = new TimedMotorStep(fl, leftPower, seconds, stop));
            if (bl != null)
                this.add(motorStep[3] = new TimedMotorStep(bl, leftPower, seconds, stop));
        }

        public void set(float leftPosition, float rightPosition) {
            motorStep[0].setPower(rightPosition);
            motorStep[1].setPower(rightPosition);
            motorStep[2].setPower(leftPosition);
            motorStep[3].setPower(leftPosition);
        }
    }


    // a Sequence that moves an up-to-four-motor robot in a straight line with given power for given encoder count
    static public class MoveByEncoderStep extends ConcurrentSequence {

        public MoveByEncoderStep(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double power, int count, boolean stop)
        {
            if (fr != null)
                this.add(new RunToPositionMotorStep(fr, power, count, stop));
            if (br != null)
                this.add(new RunToPositionMotorStep(br, power, count, stop));
            if (fl != null)
                this.add(new RunToPositionMotorStep(fl, power, count, stop));
            if (bl != null)
                this.add(new RunToPositionMotorStep(bl, power, count, stop));
        }

        public MoveByEncoderStep(DcMotor motors[], double power, int count, boolean stop)
        {
            for (DcMotor em : motors)
                if (em != null)
                    this.add(new RunToPositionMotorStep(em, power, count, stop));
        }

        public MoveByEncoderStep(OpMode mode, DcMotor motors[], float rotations, SensorLib.PID pid, float maxPower) {
            for (DcMotor em : motors)
                if (em != null)
                    this.add(new EncoderMotorStep(mode, em, rotations, pid));
        }
    }


    // a Sequence that turns an up-to-four-motor robot by applying the given right and left powers for given right and left encoder counts
    static public class TurnByEncoderStep extends ConcurrentSequence {
        RunToPositionMotorStep positionStep[];


        public TurnByEncoderStep(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double rightPower, double leftPower, int rightCount, int leftCount, boolean stop)
        {
            positionStep = new RunToPositionMotorStep[4];
            if (fr != null)
                this.add(positionStep[0] = new RunToPositionMotorStep(fr, rightPower, rightCount, stop));
            if (br != null)
                this.add(positionStep[1] = new RunToPositionMotorStep(br, rightPower, rightCount, stop));
            if (fl != null)
                this.add(positionStep[2] = new RunToPositionMotorStep(fl, leftPower, leftCount, stop));
            if (bl != null)
                this.add(positionStep[3] = new RunToPositionMotorStep(bl, leftPower, leftCount, stop));
        }

        public void set(int leftPosition, int rightPosition) {
            positionStep[0].setEncoderCount(rightPosition);
            positionStep[1].setEncoderCount(rightPosition);
            positionStep[2].setEncoderCount(leftPosition);
            positionStep[3].setEncoderCount(leftPosition);
        }

    }


    // some utilities to support "Squirrely Wheels" that move the robot sideways
    // when front and back wheels turn in opposite directions

    // example of a class used to return multiple values from a function call
    public static class MotorPowers {
        public double mFront;
        public double mBack;
        public MotorPowers(double front, double back) {
            mFront = front;
            mBack = back;
        }
        public double Front() { return mFront; }
        public double Back() { return mBack; }
    }

    // this function computes the relative front/back power settings needed to move along a given
    // heading, relative to the current orientation of the robot.
    public static MotorPowers GetSquirrelyWheelMotorPowers(
            double heading    // in degrees, zero = straight ahead, positive CCW, range +-180
    )
    {
        // wrap heading around to acceptable range
        heading = SensorLib.Utils.wrapAngle(heading);

        // compute front and back wheel relative speeds needed to go in desired direction
        double front = 0.0f;
        double back = 0.0f;
        if (heading < 0) {
            if (heading > -90) {
                front = 1.0;
                back = lerp(heading, 0, -90, 1, -1);
            } else {
                front = lerp(heading, -90, -180, 1, -1);
                back = -1.0;
            }
        }
        else {
            if (heading < 90) {
                front = lerp(heading, 0, 90, 1, -1);
                back = 1.0;
            }
            else {
                front = -1.0;
                back = lerp(heading, 90, 180, 1, -1);
            }
        }

        // return results
        return new MotorPowers(front, back);
    }

    // a Step that drives 4 "squirrely wheels" to move the robot in a given direction
    // relative to where it's facing, for a given time.
    static public class MoveSquirrelyByTimeStep extends ConcurrentSequence {

        public MoveSquirrelyByTimeStep(
                DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl,     // motors
                double heading,    // in degrees, zero = straight ahead, positive CCW, range +-180
                double power,      // overall power scaling factor
                double seconds,    // time in seconds
                boolean stop)
        {
            _MoveSquirrelyByTimeStep(fr, br, fl, bl, heading, power, seconds, stop);
        }

        public MoveSquirrelyByTimeStep(
                DcMotor motors[],  // motors -- assumed order is fr, br, fl, bl
                double heading,    // in degrees, zero = straight ahead, positive CCW, range +-180
                double power,      // overall power scaling factor
                double seconds,    // time in seconds
                boolean stop)
        {
            _MoveSquirrelyByTimeStep(motors[0], motors[1], motors[2], motors[3], heading, power, seconds, stop);
        }


        // internal function that actually does the constructor stuff for the two different ctors
        protected void _MoveSquirrelyByTimeStep(
                DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl,     // motors
                double heading,    // in degrees, zero = straight ahead, positive CCW, range +-180
                double power,      // overall power scaling factor
                double seconds,    // time in seconds
                boolean stop)
        {
            // compute relative front and back motor powers needed to move on the desired heading
            MotorPowers mp = GetSquirrelyWheelMotorPowers(heading);

            // create TimedMotorSteps to control the 4 motors
            if (fr != null)
                this.add(new TimedMotorStep(fr, mp.Front()*power, seconds, stop));
            if (br != null)
                this.add(new TimedMotorStep(br, mp.Back()*power, seconds, stop));
            if (fl != null)
                this.add(new TimedMotorStep(fl, mp.Front()*power, seconds, stop));
            if (bl != null)
                this.add(new TimedMotorStep(bl, mp.Back()*power, seconds, stop));
        }
    }

    // timer
    static public class Timer {
        long mStartTime;
        double mSeconds;

        public Timer(double seconds) {
            mStartTime = 0L;        // creation time is NOT start time
            mSeconds = seconds;
        }

        public void start() {
            mStartTime = System.nanoTime();
        }

        // return elapsed time in seconds since timer was created or restarted
        public double elapsed() {
            return (double) (System.nanoTime() - mStartTime) / (double) TimeUnit.SECONDS.toNanos(1L);
        }

        public double remaining() {
            return mSeconds - elapsed();
        }

        public boolean done() {
            return (remaining() <= 0);
        }
    }


// test hardware classes -- useful for testing when no hardware is available.
// these are primarily intended for use in testing autonomous mode code, but could
// also be useful for testing tele-operator modes.

    static public class TestHardware implements HardwareDevice {
        public HardwareDevice.Manufacturer getManufacturer() { return Manufacturer.Unknown; }
        public String getDeviceName() { return "AutoLib_TestHardware"; }
        public String getConnectionInfo() { return "connection info unknown"; }
        public int getVersion() { return 0; }
        public void resetDeviceConfigurationForOpMode() {}
        public void close() {}
    }

    // a dummy DcMotor that just logs commands we send to it --
// useful for testing Motor code when you don't have real hardware handy
    static public class TestMotor extends TestHardware implements DcMotor {
        AutoOpMode mOpMode;     // needed for logging data
        String mName;       // string id of this motor
        double mPower;      // current power setting
        DcMotor.RunMode mMode;
        int mTargetPosition;
        int mCurrentPosition;
        boolean mPowerFloat;
        Direction mDirection;
        ZeroPowerBehavior mZeroPowerBehavior;
        int mMaxSpeed;
        MotorConfigurationType mMotorType;

        public TestMotor(String name, AutoOpMode opMode) {
            super();     // init base class (real DcMotor) with dummy data
            mOpMode = opMode;
            mName = name;
            mPower = 0.0;
            mMaxSpeed = 0;
            mMode = DcMotor.RunMode.RUN_WITHOUT_ENCODERS;
            mTargetPosition = 0;
            mCurrentPosition = 0;
            mPowerFloat = false;
            mDirection = Direction.FORWARD;
            mZeroPowerBehavior = ZeroPowerBehavior.FLOAT;
        }

        @Override       // override all the functions of the real DcMotor class that touch hardware
        public void setPower(double power) {
            mPower = power;
            mOpMode.packet.put(mName, " power: " + String.valueOf(mPower));
        }

        public double getPower() {
            return mPower;
        }

        public void close() {
            mOpMode.packet.put(mName, " close();");
        }

        public boolean isBusy() {
            return false;
        }

        public void setPowerFloat() {
            mPowerFloat = true;
            mOpMode.packet.put(mName, " setPowerFloat();");
        }

        public boolean getPowerFloat() {
            return mPowerFloat;
        }

        public void setMaxSpeed(int encoderTicksPerSecond)
        {
            mMaxSpeed = encoderTicksPerSecond;
            mOpMode.packet.put(mName, "maxSpeed: " + String.valueOf(encoderTicksPerSecond));
        }

        public int getMaxSpeed() { return mMaxSpeed; }

        public void setTargetPosition(int position) {
            mTargetPosition = position;
            mOpMode.packet.put(mName, "target: " + String.valueOf(position));
        }
        public int getTargetPosition() {
            return mTargetPosition;
        }

        public int getCurrentPosition() {
            return mTargetPosition;
        }

        public void setMode(DcMotor.RunMode mode) {
            this.mMode = mode;
            mOpMode.packet.put(mName, "run mode: " + String.valueOf(mode));
        }

        public DcMotor.RunMode getMode() {
            return this.mMode;
        }

        public void setDirection(Direction direction)
        {
            mDirection = direction;
            mOpMode.packet.put(mName, "direction: " + String.valueOf(direction));
        }

        public Direction getDirection() { return mDirection; }

        public String getConnectionInfo() {
            return mName + " port: unknown";
        }

        public DcMotorController getController()
        {
            return null;
        }

        public void resetDeviceConfigurationForOpMode() {
            mOpMode.packet.put(mName, "resetDeviceConfigurationForOpMode: ");
        }

        public int getPortNumber()
        {
            return 0;
        }

        public ZeroPowerBehavior getZeroPowerBehavior()
        {
            return mZeroPowerBehavior;
        }

        public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior)
        {
            mZeroPowerBehavior = zeroPowerBehavior;
            mOpMode.packet.put(mName, "zeroPowerBehavior: " + String.valueOf(zeroPowerBehavior));
        }

        public String getDeviceName() { return "AutoLib_TestMotor: " + mName; }

        public void setMotorType(MotorConfigurationType motorType) { mMotorType = motorType; }

        public MotorConfigurationType getMotorType() { return mMotorType; }

    }

    // a dummy Servo that just logs commands we send to it --
// useful for testing Servo code when you don't have real hardware handy
    static public class TestServo extends TestHardware implements Servo {
        AutoOpMode mOpMode;     // needed for logging data
        String mName;       // string id of this servo
        double mPosition;   // current target position
        Direction mDirection;
        double mScaleMin;
        double mScaleMax;

        public TestServo(String name, AutoOpMode opMode) {
            super();     // init base class (real DcMotor) with dummy data
            mOpMode = opMode;
            mName = name;
            mPosition = 0.0;
        }

        @Override       // this function overrides the setPower() function of the real DcMotor class
        public void setPosition(double position) {
            mPosition = position;
            mOpMode.packet.put(mName, " position: " + String.valueOf(mPosition));
            mDirection = Direction.FORWARD;
        }

        @Override       // this function overrides the getPower() function of the real DcMotor class
        public double getPosition() {
            return mPosition;
        }

        @Override       // override all other functions of Servo that touch the hardware
        public String getConnectionInfo() {
            return mName + " port: unknown";
        }

        public HardwareDevice.Manufacturer getManufacturer() { return Manufacturer.Unknown; }

        public void resetDeviceConfigurationForOpMode() {
            mOpMode.packet.put(mName, "resetDeviceConfigurationForOpMode: ");
        }

        public Servo.Direction getDirection() { return mDirection; }
        public void setDirection(Servo.Direction direction) {
            mDirection = direction;
            mOpMode.packet.put(mName, "direction: " + String.valueOf(mDirection));
        }

        public void scaleRange(double min, double max)
        {
            mScaleMin = min;
            mScaleMax = max;
        }

        public ServoController getController() { return null; }
        public String getDeviceName() { return "AutoLib_TestServo: " + mName; }
        public int getPortNumber()
        {
            return 0;
        }
        public int getVersion() { return 0; }

        public void close() {}

    }

    // a dummy Gyro that just returns default info --
// useful for testing Gyro code when you don't have real hardware handy
    static public class TestGyro extends TestHardware implements GyroSensor {
        AutoOpMode mOpMode;     // needed for logging data
        String mName;       // string id of this gyro

        public TestGyro(String name, AutoOpMode opMode) {
            super();
            mOpMode = opMode;
            mName = name;
        }

        public void calibrate() {}
        public boolean isCalibrating() { return false; }
        public int getHeading() { return 0; }
        public double getRotationFraction() { return 0; }
        public int rawX() { return 0; }
        public int rawY() { return 0; }
        public int rawZ() { return 0; }
        public void resetZAxisIntegrator() {}
        public String status() { return "Status okay"; }
        public String getDeviceName() { return "AutoLib_TestGyro: " + mName; }

    }

    // a dummy ColorSensor that just returns default info --
// useful for testing ColorSensor code when you don't have real hardware handy
    static public class TestColorSensor extends TestHardware implements ColorSensor {
        AutoOpMode mOpMode;     // needed for logging data
        String mName;       // string id of this gyro

        public TestColorSensor(String name, AutoOpMode opMode) {
            super();
            mOpMode = opMode;
            mName = name;
        }

        /**
         * Get the Red values detected by the sensor as an int.
         * @return reading, unscaled.
         */
        public int red() { return 0; }

        /**
         * Get the Green values detected by the sensor as an int.
         * @return reading, unscaled.
         */
        public int green() { return 0; }

        /**
         * Get the Blue values detected by the sensor as an int.
         * @return reading, unscaled.
         */
        public int blue() { return 255; }

        /**
         * Get the amount of light detected by the sensor as an int.
         * @return reading, unscaled.
         */
        public int alpha() { return 0; }

        /**
         * Get the "hue"
         * @return hue
         */
        @ColorInt
        public int argb() { return 0; }

        /**
         * Enable the LED light
         * @param enable true to enable; false to disable
         */
        public void enableLed(boolean enable) {}

        /**
         * Set the I2C address to a new value.
         *
         */
        public void setI2cAddress(I2cAddr newAddress) {}

        /**
         * Get the current I2C Address of this object.
         * Not necessarily the same as the I2C address of the actual device.
         *
         * Return the current I2C address.
         * @return current I2C address
         */
        public I2cAddr getI2cAddress() {return I2cAddr.zero();}

        public String status() { return "Status okay"; }
        public String getDeviceName() { return "AutoLib_TestGyro: " + mName; }

    }

    // define interface to Factory that creates various kinds of hardware objects
    static public interface HardwareFactory {
        public DcMotorEx getDcMotorEx(String name);
        public DcMotor getDcMotor(String name);
        public Servo getServo(String name);
        public GyroSensor getGyro(String name);
        public ColorSensor getColorSensor(String name);
    }

    // this implementation generates test-hardware objects that just log data
    static public class TestHardwareFactory implements HardwareFactory {
        AutoOpMode mOpMode;     // needed for logging data

        public TestHardwareFactory(AutoOpMode opMode) {
            mOpMode = opMode;
        }

        public DcMotor getDcMotor(String name){
            return new TestMotor(name, mOpMode);
        }

        public DcMotorEx getDcMotorEx(String name){
            return null;
        }

        public Servo getServo(String name){
            return new TestServo(name, mOpMode);
        }

        public GyroSensor getGyro(String name){
            return new TestGyro(name, mOpMode);
        }

        public ColorSensor getColorSensor(String name){
            return new TestColorSensor(name, mOpMode);
        }
    }

    // this implementation gets real hardware objects from the hardwareMap of the given AutoOpMode
    static public class RealHardwareFactory implements HardwareFactory {
        AutoOpMode mOpMode;     // needed for access to hardwareMap

        public RealHardwareFactory(AutoOpMode opMode) {
            mOpMode = opMode;
        }

        public DcMotor getDcMotor(String name){
            DcMotor motor = null;
            try {
                motor = mOpMode.hardwareMap.dcMotor.get(name);
            }
            catch (Exception e) {
                // okay -- just return null (absent) for this motor
            }

            // just to make sure - a previous AutoOpMode may have set it differently ...
            if (motor != null)
                motor.setDirection(DcMotor.Direction.FORWARD);

            return motor;
        }

        public DcMotorEx getDcMotorEx(String name){
            DcMotorEx motor = null;
            try {
                 motor = mOpMode.hardwareMap.get(DcMotorEx.class, name);
            }
            catch (Exception e) {
                // okay -- just return null (absent) for this motor
            }

            // just to make sure - a previous AutoOpMode may have set it differently ...
            if (motor != null)
                motor.setDirection(DcMotorEx.Direction.FORWARD);

            return motor;
        }

        public Servo getServo(String name){
            Servo servo = null;
            try {
                servo = mOpMode.hardwareMap.servo.get(name);
            }
            catch (Exception e) {
                // okay - just return null (absent) for this servo
            }

            // just to make sure - a previous AutoOpMode may have set it differently ...
            if (servo != null)
                servo.setDirection(Servo.Direction.FORWARD);

            return servo;
        }

        public GyroSensor getGyro(String name){
            GyroSensor gyro = null;
            try {
                gyro = mOpMode.hardwareMap.gyroSensor.get(name);
            }
            catch (Exception e) {
                // okay - just return null (absent) for this servo
            }
            return gyro;
        }

        public ColorSensor getColorSensor(String name){
            ColorSensor cs = null;
            try {
                cs = mOpMode.hardwareMap.colorSensor.get(name);
            }
            catch (Exception e) {
                // okay - just return null (absent) for this servo
            }
            return cs;
        }

    }

    static public class MotorSetPower extends Step {
        DcMotor mMotor;
        double mPower;

        public MotorSetPower(DcMotor motor, double power) {
            mMotor = motor;
            mPower = power;
        }

        // for dynamic adjustment of power during the Step
        public void setPosition(double power) {
            mPower = power;
        }

        public boolean loop() {
            super.loop();

            mMotor.setPower(mPower);

            return true;
        }

    }

    static public class TestStep extends Step {
        float mTestValue;
        AutoOpMode mOpMode;

        public TestStep(AutoOpMode opMode, float value) {
            mOpMode = opMode;
            mTestValue = value;
        }

        public boolean loop() {
            super.loop();

            mOpMode.packet.put("Test", mTestValue);

            return false;
        }
    }
}



