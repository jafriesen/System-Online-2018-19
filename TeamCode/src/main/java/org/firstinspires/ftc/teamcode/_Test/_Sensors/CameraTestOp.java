package org.firstinspires.ftc.teamcode._Test._Sensors;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Blobs.PedestrianDetector;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_RoverRuckus;
import org.firstinspires.ftc.teamcode.Blobs.blobtrack.MovingBlob;
import org.firstinspires.ftc.teamcode.Blobs.image.Color;


import java.util.ArrayList;
import java.util.List;

/**
 * Created by joshua on 18/10/10.
 */

@Autonomous(name="Test: CameraTest", group="Test")
public class CameraTestOp extends OpMode {

    VuforiaLib_RoverRuckus vLib;

    int pixels[];
    byte simple[];
    PedestrianDetector cubeDetect;
    int counter;

    @Override
    public void init(){
        vLib = new VuforiaLib_RoverRuckus();
        vLib.init(this, null);
        pixels = new int[640*480];
        simple = new byte[640*480];
        cubeDetect = new PedestrianDetector();
        msStuckDetectLoop = 1000000;
        counter = 0;
    }

    @Override
    public void start(){
        vLib.start();
    }

    @Override
    public void loop(){
        vLib.loop(true);
        Bitmap bm = vLib.getBitmap(4);
        int bmArray[] = new int[bm.getWidth()*bm.getHeight()];
        if(counter % 5 == 0) {
            bm.getPixels(bmArray, 0, bm.getWidth(), 0, 0, bm.getWidth(), bm.getHeight());
            if (bm.getHeight() / 480 > bm.getWidth() / 640) {
                scaleDown(bmArray, pixels, bm.getHeight() / 480);
            } else {
                scaleDown(bmArray, pixels, bm.getWidth() / 640);
            }
            telemetry.addData("Top Left Pixel", String.format("0x%08x", pixels[0]));
            this.convertToSimpleColorRaster(pixels, simple, 480, 640, 640);
            List<MovingBlob> blobs = this.cubeDetect.getAllBlobs(simple, 640);
            ArrayList<MovingBlob> yellowBlobs = new ArrayList<MovingBlob>();
            telemetry.addData("Number of Blobs", blobs.size());
            for(MovingBlob blob : blobs){
                if(blob.color.getColor() == Color.YELLOW){
                    yellowBlobs.add(blob);
                }
            }
            telemetry.addData("No of Yellow Blobs", yellowBlobs.size());
        }
    }

    public static void convertToSimpleColorRaster(int rgb[], byte[] simple, int nrows, int ncols, int frameWidth) {
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
                int red = ((rgb[ncols*r+c]%0x100000)/0x1000)&0xFF;
                int green = ((rgb[ncols*r+c]%0x1000)/0x10)&0xFF;
                int blue = (rgb[ncols*r+c]%0x10)&0xFF;
                //int B = (((int)bayer[(r*ncols*2 + c)*2 + 1+2*ncols-ncols*2*getBit(tile,1)-getBit(tile,0)])&0xFF);			//Bottom right (blue)
                double Y = red *  .299000 + green *  .587000 + blue *  .114000;
                double U  = red * -.168736 + green * -.331264 + blue *  .500000 + 128;
                double V = red *  .500000 + green * -.418688 + blue * -.081312 + 128;
                red =(int)(  1.4075 * (V - 128));
                green = (int)(0- 0.3455 * (U - 128) - (0.7169 * (V - 128)));
                blue = (int)(1.7790 * (U - 128));
                //If one of the colors has a value 50 greater than both other colors
                //it assigns that pixel to that color
                if(red > green+50 && red > blue+50){
                    simple[r*ncols+c] = 0;
                } else if(green > red+40 && green > blue-10){
                    simple[r*ncols+c] = 1;
                } else if(blue > red+50 && blue > green+50){
                    simple[r*ncols+c] = 2;
                }else if(red<green+25&&green<red+25&&(red>blue+70)){
                    simple[r*ncols+c] = 6;
                }
                //Otherwise it sees if one of the colors has a value above 170 for white
                // if not, 85 for grey and below 85 for black
                else if(Y>140){
                    simple[r*ncols+c] = 3;
                } else if(Y>85){
                    simple[r*ncols+c] = 4; //0x808080
                } else {
                    simple[r * ncols + c] = 5;
                }
            }
        }
    }

    public static void scaleDown(int input[], int output[], int scale){
        try {
            for (int i = 0; i < input.length; i++) {
                output[i] = input[i * scale];
            }
        }catch(Exception e){

        }
    }

    public boolean detectCube(MovingBlob blob){
        if(blob.height - blob.width < 25 &&
            blob.color.getColor() == Color.YELLOW &&
                blob.y < 380 && blob.y > 100   ){
            blob.seen = true;
            return true;
        }
        return false;
    }

}


