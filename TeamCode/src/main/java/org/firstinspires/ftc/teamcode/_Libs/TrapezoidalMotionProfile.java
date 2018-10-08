package org.firstinspires.ftc.teamcode._Libs;

public class TrapezoidalMotionProfile {
    private float mTargetPos;
    private double mMaxVel;
    private double mMaxAccel;

    private double mETA;             // time to arrive at target position
    private double mJerk1, mJerk2;   // times where jerk is applied; "corners" of the trapezoid

    private double mStartTime;
    private double mElapsedTime;

    private double mVelocity;
    private double mPosition;

    private boolean mDone = false;
    private boolean mFirstTime = true;

    public TrapezoidalMotionProfile() {}

    public TrapezoidalMotionProfile(float distance, float velocity, float acceleration) {
        mTargetPos = distance;
        mMaxVel = velocity;
        mMaxAccel = acceleration;

        mJerk1 = mMaxVel / mMaxAccel;
        mETA = mJerk1 + mTargetPos / mMaxVel;
        mJerk2 = mETA - mJerk1;
    }

    public void update(double time) {
        if(mFirstTime) {
            mStartTime = time;
            mFirstTime = false;
        }

        mElapsedTime = (time - mStartTime);

        if(mElapsedTime < mETA) {
            if (mElapsedTime < mJerk1) {
                mVelocity = mMaxAccel * mElapsedTime;
                mPosition = mMaxAccel / 2 * (mElapsedTime * mElapsedTime);
            } else if (mElapsedTime <= mJerk2) {
                mVelocity = mMaxVel;
                mPosition = mMaxVel * (mElapsedTime - mJerk1) + (mMaxVel*mMaxVel/(2*mMaxAccel));
            } else if (mElapsedTime > mJerk2) {
                mVelocity = -mMaxAccel * (mElapsedTime - mETA);
                mPosition = -mMaxAccel / 2 * (mElapsedTime - mETA) * (mElapsedTime - mETA) + mETA * mMaxVel - (mMaxVel * mMaxVel / mMaxAccel);
            }
        }
        else {
            mDone = true;
        }
    }

    public double getVelocity() {
        return mVelocity;
    }

    public double getPosition() {
        return mPosition;
    }

    public double getElapsedTime() {
        return mElapsedTime;
    }

    public boolean isDone() {
        return mDone;
    }

    public void setTargetPos(int rotations) {
        mTargetPos = rotations;
    }

    public void setMaxVelocity(float velocity) {
        mMaxVel = velocity;
    }

    public void setMaxAcceleration(float acceleration) {
        mMaxAccel = acceleration;
    }
}
