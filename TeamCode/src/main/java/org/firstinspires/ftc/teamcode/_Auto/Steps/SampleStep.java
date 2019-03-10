package org.firstinspires.ftc.teamcode._Auto.Steps;


import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Blobs.image.Color;
import org.firstinspires.ftc.teamcode.Blobs.image.Pixel;
import org.firstinspires.ftc.teamcode.Blobs.blobdetect.BlobDetection;
import org.firstinspires.ftc.teamcode.Blobs.blobdetect.Blob;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_RoverRuckus;

import java.util.LinkedList;
import java.util.List;

public class SampleStep extends AutoLib.Step {

    VuforiaLib_RoverRuckus mVLib;
    OpMode opMode;
    int cubePositionCount[], cubePosition;
    AutoLib.Timer mTimer;
    boolean save;
    AutoLib.Data data;
    BlobDetection blobDetector;

    public SampleStep(VuforiaLib_RoverRuckus mVLib, OpMode opMode) {
        this.mVLib = mVLib;
        this.opMode = opMode;
        this.cubePositionCount = new int[3];
        this.save = false;
        blobDetector = new BlobDetection();
    }

    public SampleStep(VuforiaLib_RoverRuckus mVLib, OpMode opMode, AutoLib.Data data){
        this.mVLib = mVLib;
        this.opMode = opMode;
        this.data = data;
        this.cubePositionCount = new int[3];
        this.save = true;
        mTimer = new AutoLib.Timer(1.0);
        blobDetector = new BlobDetection();
    }

    @Override
    public boolean loop() {
        Bitmap bmIn = mVLib.getBitmap(4);
        int bmArray[] = new int[bmIn.getWidth()*bmIn.getWidth()];
        int bmOutArray[] = new int[bmIn.getWidth()*bmIn.getHeight()];
        Pixel pixels[][] = new Pixel[bmIn.getHeight()][bmIn.getWidth()];
        List<Blob> blobs = new LinkedList<Blob>();
        int cubeX = 0;

        if(firstLoopCall() && mTimer != null) {
            mTimer.start();
        }

        if (bmIn != null) {
            // create the output bitmap we'll display on the RC phone screen
            bmIn.getPixels(bmArray, 0, bmIn.getWidth(), 0, 0, bmIn.getWidth(), bmIn.getHeight());

            convertToSimpleColorRaster(bmArray, bmOutArray, bmIn.getHeight(), bmIn.getWidth(), bmIn.getWidth());
            convertToPixels(bmOutArray, pixels, bmIn.getWidth(), bmIn.getHeight());

            blobs = blobDetector.getBlobs(pixels);

            int whiteAvgX = 0;
            boolean foundGold = false;
            for(int i = 0; i < blobs.size(); i++) {
                if (blobs.get(i).color.getColor() == Color.YELLOW && blobs.get(i).width > 15 && blobs.get(i).width < 40 && blobs.get(i).height > 25) {
                    cubeX = blobs.get(i).x;
                    if (cubeX < bmIn.getWidth() / 3) {
                        cubePositionCount[2]++;
                    } else if (cubeX > bmIn.getWidth() / 3 && cubeX < bmIn.getWidth() * 2 / 3) {
                        cubePositionCount[1]++;
                    } else if (cubeX > bmIn.getWidth() * 2 / 3) {
                        cubePositionCount[0]++;
                    } else {
                        cubePositionCount[0]++;
                    }
                    foundGold = true;
                }

                if (blobs.get(i).color.getColor() == Color.WHITE && blobs.get(i).width > 15 && blobs.get(i).width < 40) {
                    if (whiteAvgX == 0) {
                        whiteAvgX += blobs.get(i).x;
                    } else {
                        whiteAvgX += blobs.get(i).x;
                        whiteAvgX /= 2;
                    }
                }

            }
            if(!foundGold){
                if(whiteAvgX > bmIn.getWidth()/2){
                    cubePositionCount[2]++;
                }else if(whiteAvgX < bmIn.getWidth()/2){
                    cubePositionCount[0]++;
                }

            }



            if(cubePositionCount[2] > cubePositionCount[1] && cubePositionCount[2] > cubePositionCount[0]) {
                cubePosition = 3;
            }
            else if(cubePositionCount[1] > cubePositionCount[0] && cubePositionCount[1] > cubePositionCount[2]) {
                cubePosition = 2;
            }
            else {
                cubePosition = 1;
            }

            opMode.telemetry.addData("Cube X", cubePosition);

            if(mTimer != null && mTimer.done() && save) {
                data.Float = cubePosition;
                return true;
            }
        }
        return super.loop();
    }

    public static void convertToSimpleColorRaster(int rgb[], int[] simple, int nrows, int ncols, int frameWidth) {

        for(int r = 0; r < nrows; r++){
            for(int c = 0; c < frameWidth; c++){
                int red = (rgb[ncols*r+c]&0xFF0000)>>16;
                int green = (rgb[ncols*r+c]&0xFF00)>>8;
                int blue = (rgb[ncols*r+c]&0xFF);

                double Y = red *  .299000 + green *  .587000 + blue *  .114000;
                double U  = red * -.168736 + green * -.331264 + blue *  .500000 + 128;
                double V = red *  .500000 + green * -.418688 + blue * -.081312 + 128;
                if(r > nrows * 2/3){
                    simple[ncols*r+c] = 0xFF000000;
                }else if(red > 2.25*blue && green > 1.75*blue && red > 80){
                    simple[r*ncols+c] = 0xFFFFFF00;
                }else {
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


}

