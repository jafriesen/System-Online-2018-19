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

import org.firstinspires.ftc.teamcode.Blobs.blobdetect.Blob;
import org.firstinspires.ftc.teamcode.Blobs.image.Color;
import org.firstinspires.ftc.teamcode._Libs.CameraLib;
//import org.firstinspires.ftc.teamcode._Libs.RS_Posterize;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_RoverRuckus;
import org.firstinspires.ftc.teamcode.Blobs.blobdetect.BlobDetection;
import org.firstinspires.ftc.teamcode.Blobs.image.Pixel;

import java.util.LinkedList;
import java.util.List;

import static java.lang.Math.abs;


@Autonomous(name="Test: PosterizeTest", group ="Test")
//@Disabled
public class PosterizeTestOp extends OpMode {

    int mLoopCount;
    VuforiaLib_RoverRuckus mVLib;
    ImageView mView;
    //RS_Posterize mRsPosterize;
    Bitmap mBmOut;
    BlobDetection blobDetector;

    enum PosterColor {
        RED(0xFF0000, (short)0),
        GREEN(0x00FF00, (short)1),
        BLUE(0x0000FF, (short)2),
        CYAN(0x00FFFF, (short)3),
        MAGENTA(0xFF00FF, (short)4),
        YELLOW(0xFFFF00, (short)5),
        BLACK(0, (short)6),
        GREY1(0x333333, (short)7),
        GREY2(0x666666, (short)8),
        GREY3(0x999999, (short)9),
        GREY4(0xCCCCCC, (short)10),
        WHITE(0xFFFFFF, (short)11);
        final int rgb;
        final short code;
        private PosterColor(int rgb, short code) {
            this.rgb = rgb;
            this.code = code;
        }
    }

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
        blobDetector = new BlobDetection();


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
        //Pixel pixels[][] = new Pixel[bmIn.getHeight()][bmIn.getWidth()];
        int cubeX = 0;
        int cubePosition = 0;

        List<Blob> blobs = new LinkedList<>();
        if (bmIn != null) {
            // create the output bitmap we'll display on the RC phone screen
            mBmOut = Bitmap.createBitmap(bmIn.getWidth(), bmIn.getHeight(), Bitmap.Config.RGB_565);
            bmIn.getPixels(bmArray,0,bmIn.getWidth(),0,0,bmIn.getWidth(),bmIn.getHeight());
            for(int i = 0; i < bmIn.getWidth()*bmIn.getHeight(); i++){
                bmOutArray[i] = posterizePixelHSL(bmArray[i],65).rgb;
            }
            //convertToSimpleColorRaster(bmArray,bmOutArray,bmIn.getHeight(),bmIn.getWidth(),bmIn.getWidth());convertToPixels(bmOutArray, pixels,bmIn.getWidth(),bmIn.getHeight());
            mBmOut.setPixels(bmOutArray,0,bmIn.getWidth(),0,0,bmIn.getWidth(),bmIn.getHeight());
            /*-
            blobs = blobDetector.getBlobs(pixels);
            boolean foundGold = false;
            int whiteAvgX = 0;
            for(int i = 0; i < blobs.size(); i++){
                if(blobs.get(i).color.getColor() == Color.YELLOW && blobs.get(i).width > 15 && blobs.get(i).width < 40 && blobs.get(1).height > 15){
                    cubeX = blobs.get(i).x;
                    if(cubeX < bmIn.getWidth() /2){
                        cubePosition = 3;
                    }else if(cubeX > bmIn.getWidth()/2){
                        cubePosition = 2;
                    }
                    foundGold = true;
                    telemetry.addData("Cube Width", blobs.get(i).width);
                }
                if(blobs.get(i).color.getColor() == Color.WHITE && blobs.get(i).width > 15 && blobs.get(i).width < 40){
                    if(whiteAvgX == 0){
                        whiteAvgX += blobs.get(i).x;
                    }else{
                        whiteAvgX += blobs.get(i).x;
                        whiteAvgX /= 2;
                    }
                }
            }
            if(!foundGold){
                if(whiteAvgX > bmIn.getWidth()/2){
                    cubePosition = 3;
                }else if(whiteAvgX < bmIn.getWidth()/2){
                    cubePosition = 1;
                }
                telemetry.addData("Silver Avg", whiteAvgX);

            }

            telemetry.addData("Sample", cubePosition);
            */

            /*
            for(int i = 0; i < blobs.size(); i++){
                int hwratio = blobs.get(i).width / blobs.get(i).height;
                if(blobs.get(i).color.getColor() == Color.BLACK|| blobs.get(i).color.getColor() == Color.WHITE ){
                    blobs.remove(i);
                }else if(hwratio < 0.75 || hwratio > 1.33333){
                    //blobs.remove(i);
                }
                else {
                    telemetry.addData("Cube X", blobs.get(i).width);
                    telemetry.addData("Distance", 12.0 * (44.0/(double)blobs.get(i).width));
                }
            }
            *.

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
                if(Y > 0xE0){
                    simple[r*ncols+c] = 0xFFFFFFFF;
                }else if(U < 128 && V < 154){
                    simple[r*ncols+c] = 0xFF00FF00;
                }else if(U > 179){
                    simple[r*ncols+c] = 0xFF0000FF;
                } else if(U < 179 && V > 154){
                    simple[r*ncols+c] = 0xFFFF0000;
                } else {
                    //simple[ncols * r + c] = (int) Y | (int) Y << 8 | (int) Y << 16 | 0xFF000000;
                    simple[ncols*r+c] = 0xFF000000;
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

    public void erosion(int input[], int output[], int ncols, int nrows, int color){
        for(int r = 0; r < nrows; r++){
            for(int c = 0; c < ncols; c++){
                if(input[ncols*r + c] == color){
                    int neigboring = 0;
                    if(input[ncols*r + c +1] == color){
                        neigboring++;
                    }
                    if(input[ncols*r + c -1] == color){
                        neigboring++;
                    }
                    if(input[ncols*(r+1) + c] == color){
                        neigboring++;
                    }
                    if(input[ncols*(r-1) + c] == color){
                        neigboring++;
                    }
                    if(neigboring > 1){
                        output[ncols*r + c] = input[ncols*r + c];
                    }else{
                        output[ncols*r + c] = 0xFF000000;
                    }
                } else{
                    output[ncols*r + c] = input[ncols*r + c];
                }
            }
        }
    }

    public void convertToPixels(int input[], Pixel output[][], int ncols, int nrows){
        for(int r = 0; r < nrows; r++){
            for(int c = 0; c < ncols; c++){
                switch (input[ncols*r+c]){
                    case 0xFF000000:
                        output[r][c] = new Pixel(Color.BLACK);
                        break;
                    case 0xFFFFFF00:
                        output[r][c] = new Pixel(Color.YELLOW);
                        break;
                    case 0xFFFFFFFF:
                        output[r][c] = new Pixel(Color.WHITE);
                        break;
                        default:
                            output[r][c] = new Pixel(Color.BLACK);
                            break;


                }
            }
        }
    }

    static PosterColor posterizePixel(int rgb, int dt) {
        int red = (rgb >> 16) & 0xFF;
        int green = (rgb >> 8) & 0xFF;
        int blue = (rgb) & 0xFF;
        int rg = red - green;
        int rb = red - blue;
        int bg = blue - green;
        if(rg > dt && rb > dt) {
            return PosterColor.RED;
        } else if (rg < -dt && bg < -dt) {
            return PosterColor.GREEN;
        } else if (rb < -dt && bg > dt) {
            return PosterColor.BLUE;
        } else if (rg < dt && rg > -dt && rb > dt && bg < -dt) {
            return PosterColor.YELLOW;
        } else if (rb < dt && rb > -dt && rg > dt && bg > dt) {
            return PosterColor.MAGENTA;
        } else if (bg < dt && bg > -dt && rg < -dt && rb < -dt ) {
            return PosterColor.CYAN;
        } else {
            int avg = (red + green + blue + green) >> 2;
            if(avg < 25) {
                return PosterColor.BLACK;
            } else if(avg < 76) {
                return PosterColor.GREY1;
            } else if(avg < 127) {
                return PosterColor.GREY2;
            } else if(avg < 178) {
                return PosterColor.GREY3;
            } else if(avg < 229) {
                return PosterColor.GREY4;
            } else {
                return PosterColor.WHITE;
            }

        }
    }

    static PosterColor posterizePixelHSL(int rgb, int dt) {
        int red = (rgb >> 16) & 0xFF;
        int green = (rgb >> 8) & 0xFF;
        int blue = (rgb) & 0xFF;
        int max = red > blue ? red > green ? red : green : blue > green ? blue : green;
        int min = red < blue ? red < green ? red : green : blue < green ? blue : green;
        int delta = max - min;
        int h = 0;
        if(delta == 0){
            h = 0;
        }else if(max == red){
            h = ((green-blue)/delta) % 6;
        }else if(max == green){
            h = (blue - red)/delta + 2;
        }else{
            h = (red - green)/delta + 4;
        }
        h *= 60;
        int l = (max + min) >> 1;
        if(delta > dt){
            if(h > 330 || h < 30){
                return PosterColor.RED;
            }else if(h > 30 && h < 90){
                return PosterColor.YELLOW;
            }else if( h > 90 && h < 150){
                return PosterColor.GREEN;
            }else if(h > 150 && h < 210){
                return PosterColor.CYAN;
            }else if(h > 210 && h < 270){
                return PosterColor.BLUE;
            }else if(h > 270 && h < 330){
                return PosterColor.MAGENTA;
            }
        }else{
            if(l < 43){
                return PosterColor.BLACK;
            }else if(l > 43 && l < 86){
                return  PosterColor.GREY1;
            }else if(l > 86 && l < 129){
                return PosterColor.GREY2;
            }else if(l > 129 && l < 152){
                return PosterColor.GREY3;
            }else if(l > 152 && l < 195){
                return PosterColor.GREY4;
            }else{
                return PosterColor.WHITE;
            }
        }
        return PosterColor.BLACK;
    }



}
