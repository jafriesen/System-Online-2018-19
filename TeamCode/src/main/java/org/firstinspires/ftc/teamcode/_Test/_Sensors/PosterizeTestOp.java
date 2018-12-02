/**
 * CameraTestOp - a simple operating mode that tries to acquire an image from the
 * phone camera and get some data from the image
 * Created by phanau on 12/9/15.
 */

package org.firstinspires.ftc.teamcode._Test._Sensors;

import android.app.Activity;
import android.graphics.Bitmap;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._Libs.CameraLib;
//import org.firstinspires.ftc.teamcode._Libs.RS_Posterize;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_RoverRuckus;

import static java.lang.Math.abs;


@Autonomous(name="Test: PosterizeTest", group ="Test")
//@Disabled
public class PosterizeTestOp extends OpMode {

    int mLoopCount;
    VuforiaLib_RoverRuckus mVLib;
    ImageView mView;
    //RS_Posterize mRsPosterize;
    Bitmap mBmOut;

    // Constructor
    public PosterizeTestOp() {
    }

    @Override
    public void init() {
        mVLib = new VuforiaLib_RoverRuckus();
        mVLib.init(this, null);     // pass it this OpMode (so it can do telemetry output) and use its license key for now

        mView = (ImageView)((Activity)hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.OpenCVOverlay);
        mView.post(new Runnable() {
            @Override
            public void run() {
                mView.setAlpha(1.0f);
            }
        });

    }

    @Override public void start()
    {
        // do this here -- takes too long to do in init()??
        //mRsPosterize = new RS_Posterize();
        //mRsPosterize.createScript(this.hardwareMap.appContext);

        /** Start tracking the data sets we care about. */
        mVLib.start();
    }

    public void loop() {

        // test image access through Vuforia
        Bitmap bmIn = mVLib.getBitmap(4);
        int bmArray[] = new int[bmIn.getWidth()*bmIn.getWidth()];
        int bmOutArray[] = new int[bmIn.getWidth()*bmIn.getHeight()];
        int cubeX = 0;
        int cubePosition = 0;
        if (bmIn != null) {
            // create the output bitmap we'll display on the RC phone screen
            mBmOut = Bitmap.createBitmap(bmIn.getWidth(), bmIn.getHeight(), Bitmap.Config.RGB_565);
            bmIn.getPixels(bmArray,0,bmIn.getWidth(),0,0,bmIn.getWidth(),bmIn.getHeight());
            convertToSimpleColorRaster(bmArray,bmOutArray,bmIn.getHeight(),bmIn.getWidth(),bmIn.getWidth());
            mBmOut.setPixels(bmOutArray,0,bmIn.getWidth(),0,0,bmIn.getWidth(),bmIn.getHeight());
            cubeX = findCube(bmOutArray, bmIn.getHeight(), bmIn.getWidth());
            if(cubeX < bmIn.getWidth()/3){
                cubePosition = 1;
            }else if(cubeX > bmIn.getWidth() * 2/3){
                cubePosition = 3;
            }else{
                cubePosition = 2;
            }

            telemetry.addData("Input Center", String.format("0x%08x", bmArray[bmIn.getWidth()*bmIn.getHeight()/2]));
            telemetry.addData("Output Center", String.format("0x%08x", bmOutArray[bmIn.getWidth()*bmIn.getHeight()/2]));
            telemetry.addData("Cube X", cubePosition);
            // do some processing on the input bitmap in RenderScript to generate the output image
            //mRsPosterize.runScript(bmIn, mBmOut);

            // do some data extraction on the processed bitmap
            /*
            CameraLib.CameraImage frame = new CameraLib.CameraImage(bmIn);
            CameraLib.Size camSize = frame.cameraSize();
            telemetry.addData("Size", String.valueOf(camSize.width) + "x" + String.valueOf(camSize.height));
            final int bandSize = 2;         // width of each band of pixel columns below
            final float minFrac = 0.4f;     // minimum fraction of pixels in band that needs to be of same color to mark it as "dominant"
            telemetry.addData("hue columns", frame.columnHue(bandSize, null, minFrac));
            telemetry.addData("dom columns", frame.columnDomColor(bandSize, null, minFrac));
            */

            //display the processed bitmap
            mView.post(new Runnable() {
                @Override
                public void run() {
                    //synchronized (bmLock) {
                    mView.setImageBitmap(mBmOut);
                    mView.invalidate();
                    //}
                }
            });
        }
    }

    public void stop() {
        //if (mRsPosterize != null)
            //mRsPosterize.destroyScript();
        mVLib.stop();       // Vuforia claims to do this automatically on OpMode stop but ...
        mView.post(new Runnable() {
            @Override
            public void run() {
                mView.setAlpha(0.0f);
            }
        });     // hide the overlay window
    }

    public static void convertToSimpleColorRaster(int rgb[], int[] simple, int nrows, int ncols, int frameWidth) {
        /*
         *d
         *Serves color raster encoded in 1D of values 0-5 with
         * 0 = RED
         * 1 = GREEN
         * 2 = BLUE
         * 3 = WHITE
         * 4 = GREY
         * 5 = BLACK
         * 6 = YELLOW
         */
        for(int r = 0; r < nrows; r++){
            for(int c = 0; c < frameWidth; c++){
                int red = (rgb[ncols*r+c]&0xFF0000)>>16;
                int green = (rgb[ncols*r+c]&0xFF00)>>8;
                int blue = (rgb[ncols*r+c]&0xFF);
                //int B = (((int)bayer[(r*ncols*2 + c)*2 + 1+2*ncols-ncols*2*getBit(tile,1)-getBit(tile,0)])&0xFF);			//Bottom right (blue)

                double Y = red *  .299000 + green *  .587000 + blue *  .114000;
                double U  = red * -.168736 + green * -.331264 + blue *  .500000 + 128;
                double V = red *  .500000 + green * -.418688 + blue * -.081312 + 128;
                if(r <  nrows / 2){
                    simple[ncols*r+c] = 0xFF000000;
                }else if(red > 2.25*blue && green > 1.75*blue && red > 80){
                    simple[r*ncols+c] = 0xFFFFFF00;
                }else {
                    simple[ncols * r + c] = (int) Y | (int) Y << 8 | (int) Y << 16 | 0xFF000000;
                    //simple[ncols*r+c] = 0xFF000000;
                }
                /*
                red =(int)(  1.4075 * (V - 128));
                green = (int)(0- 0.3455 * (U - 128) - (0.7169 * (V - 128)));
                blue = (int)(1.7790 * (U - 128));
                */
                //If one of the colors has a value 50 greater than both other colors
                //it assigns that pixel to that color
                /*
                if(red > green+50 && red > blue+50){
                    simple[r*ncols+c] = 0xFFFF0000;
                } else if(green > red+40 && green > blue-10){
                    simple[r*ncols+c] = 0xFF00FF00;
                } else if(blue > red+50 && blue > green+50){
                    simple[r*ncols+c] = 0xFF0000FF;
                }else if(red<green+25&&green<red+25&&(red>blue+70)){
                    simple[r*ncols+c] = 0xFFFFFF00;
                }
                //Otherwise it sees if one of the colors has a value above 170 for white
                // if not, 85 for grey and below 85 for black
                else if(Y>140){
                    simple[r*ncols+c] = 0xFFFFFFFF;
                } else if(Y>85){
                    simple[r*ncols+c] = 0xFF808080; //0x808080
                } else {
                    simple[r * ncols + c] = 0xFF000000;
                }
                */
            }
        }
    }

    public int findCube(int input[], int nrows, int ncols){
        int averageX[] = new int[nrows];
        int total = 0;
        int counted = 0;
        for(int r = 0; r < nrows; r++ ){
            int averageRowX = 0;
            int numYellow = 0;
            for(int c = 0; c < ncols; c++){
                if(input[r*ncols+c] == 0xFFFFFF00){
                    averageRowX += c;
                    numYellow++;
                }
            }
            if(numYellow > 0){
                averageX[r] = averageRowX/numYellow;
            }
        }
        for(int i = 0; i < averageX.length; i++){
            if(averageX[i] > 0){
                total += averageX[i];
                counted++;
            }
        }
        if(counted > 0) {
            return total / counted;
        } else{
            return 0;
        }
    }

}
