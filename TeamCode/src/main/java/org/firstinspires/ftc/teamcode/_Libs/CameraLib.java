package org.firstinspires.ftc.teamcode._Libs;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.Point;
import android.graphics.Rect;
import android.graphics.RectF;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;

import java.nio.IntBuffer;
import java.util.List;

import static org.firstinspires.ftc.teamcode._Libs.CameraLib.colors.eBlue;
import static org.firstinspires.ftc.teamcode._Libs.CameraLib.colors.eCyan;
import static org.firstinspires.ftc.teamcode._Libs.CameraLib.colors.eGreen;
import static org.firstinspires.ftc.teamcode._Libs.CameraLib.colors.eMagenta;
import static org.firstinspires.ftc.teamcode._Libs.CameraLib.colors.eRed;
import static org.firstinspires.ftc.teamcode._Libs.CameraLib.colors.eYellow;

/**
 * Library of utility classes that support acquiring images from the phone's camera
 * Created by phanau on 12/30/15.
 */
public class CameraLib {

    // functional utility class that converts image in NV21 format to RGB
    static class NV21toRGB {

        static Bitmap convert(byte[] data, int imageWidth, int imageHeight) {

            // the bitmap we want to fill with the image
            Bitmap bitmap = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.ARGB_8888);
            int numPixels = imageWidth * imageHeight;

            // the buffer we fill up which we then fill the bitmap with
            IntBuffer intBuffer = IntBuffer.allocate(imageWidth * imageHeight);
            // If you're reusing a buffer, next line imperative to refill from the start,
            // if not good practice
            intBuffer.position(0);

            // Set the alpha for the image: 0 is transparent, 255 fully opaque
            final byte alpha = (byte) 255;

            // Get each pixel, one at a time
            for (int y = 0; y < imageHeight; y++) {
                for (int x = 0; x < imageWidth; x++) {
                    // Get the Y value, stored in the first block of data
                    // The logical "AND 0xff" is needed to deal with the signed issue
                    int Y = data[y * imageWidth + x] & 0xff;

                    // Get U and V values, stored after Y values, one per 2x2 block
                    // of pixels, interleaved. Prepare them as floats with correct range
                    // ready for calculation later.
                    int xby2 = x / 2;
                    int yby2 = y / 2;

                    // change this to V for NV12/420SP
                    float U = (float) (data[numPixels + 2 * xby2 + yby2 * imageWidth] & 0xff) - 128.0f;

                    // change this to U for NV12/420SP
                    float V = (float) (data[numPixels + 2 * xby2 + 1 + yby2 * imageWidth] & 0xff) - 128.0f;

                    // Do the YUV -> RGB conversion
                    float Yf = 1.164f * ((float) Y) - 16.0f;
                    int R = (int) (Yf + 1.596f * V);
                    int G = (int) (Yf - 0.813f * V - 0.391f * U);
                    int B = (int) (Yf + 2.018f * U);

                    // Clip rgb values to 0-255
                    R = R < 0 ? 0 : R > 255 ? 255 : R;
                    G = G < 0 ? 0 : G > 255 ? 255 : G;
                    B = B < 0 ? 0 : B > 255 ? 255 : B;

                    // Put that pixel in the buffer
                    intBuffer.put((alpha << 24) + (R << 16) + (G << 8) + B);
                }
            }

            // Get buffer ready to be read
            intBuffer.flip();

            // Push the pixel information from the buffer onto the bitmap.
            bitmap.copyPixelsFromBuffer(intBuffer);

            return bitmap;
        }
    }

    public static class Size {
        public int width;
        public int height;
        public Size(int w, int h) {
            width = w;
            height = h;
        }
    };

    public enum colors { eWhite, eRed, eYellow, eGreen, eCyan, eBlue, eMagenta, eGray };

    // a simple utility class that provides a few more operations on an RGB pixel encoded as an int
    // by extending the Android Color class that does most of what we need.
    public static class Pixel extends Color {
        static float[] mHSV = new float[3];     // scratch storage - allocated once at start-up

        public static String toString(int rgb) {
            return "pixel("+Color.red(rgb)+","+green(rgb)+","+blue(rgb)+")";
        }

        // return hue of given RGB pixel, discretized to 6 principal colors:
        // Red,Yellow,Green,Cyan,Blue,Magenta (1..6) or, if saturation < threshold, one of 8 gray levels
        public static int hue(int pix) {
            float[] hsv = RGBtoHSV(pix, mHSV);
            if (hsv[1] < 0.20) {
                return (int)(hsv[2] * 7 + 7);       // return Value discretized to 8 levels (7..14)
            }
            int iHue = ((int)hsv[0]+30)/60;         // round to nearest 60 degrees of hue
            if (iHue == 6)
                iHue = 0;                           // red is either 0 or 360 degrees of hue
            return iHue+1;                          // return discretized hue/gray RYGCBM01234567
        }

        // return integer grayscale value (0:255) of a given RGB pixel
        public static int value(int pix){
            float[] hsv = RGBtoHSV(pix, mHSV);
            return (int)(hsv[2]*255);
        }

        // return "dominant color" of an RGB pixel if it has one (or white if it doesn't)
        public static int dominantColor(int pix) {
            final float n = 1.5F;    // dominance factor threshold
            int domClr = 0;     // default is white (i.e. shades of gray)
            if (red(pix)>n*green(pix) && red(pix)>n*blue(pix))
                domClr = eRed.ordinal();     // red
            else
            if (green(pix)>n*red(pix) && green(pix)>n*blue(pix))
                domClr = eGreen.ordinal();     // green
            else
            if (blue(pix)>n*red(pix) && blue(pix)>n*green(pix))
                domClr = eBlue.ordinal();     // blue
            else
            if (blue(pix)>n*red(pix) && green(pix)>n*red(pix))
                domClr = eCyan.ordinal();     // cyan
            else
            if (blue(pix)>n*green(pix) && red(pix)>n*green(pix))
                domClr = eMagenta.ordinal();     // magenta
            else
            if (red(pix)>n*blue(pix) && green(pix)>n*blue(pix))
                domClr = eYellow.ordinal();     // yellow
            // if it has no discernible hue, encode its gray level 0-7
            if (domClr == 0) {
                float value = red(pix)*0.2f + green(pix)*0.7f + blue(pix)*0.1f; // 0..255
                domClr = (int)(value / 32 + 7);  // return Value discretized to 8 levels (7..14)
            }
            return domClr;
        }

        // return string name of given color index
        public static String colorName(int i) {
            String c[] = {"w", "r", "y", "g", "c", "b", "m", "0","1","2","3","4","5","6","7"};
            return c[i];
        }

        public static float[] RGBtoHSV(int pix, float[] hsv) {
            // caller passes in return storage (hsv) to avoid lots of allocations
            // Convert RGB components to HSV.
            // hsv[0] is Hue [0 .. 360)
            // hsv[1] is Saturation [0...1]
            // hsv[2] is Value [0...1]
            colorToHSV(pix, hsv);
            return hsv;     // (also return arg2 for convenience)
        }

        public static int HSVtoRGB(float[] hsv) {
            return HSVToColor(hsv);
        }
    }

    // filter interface ---
    // client can use this to collapse the full set of recognized hues and brightnesses to a smaller set.
    // e.g. if you want both blue and cyan to be coded as "blue".
    public interface Filter {
        public int map(int i);
    }

    // posterizer classes ---
    // each takes an RGB pixel value and returns an enumerated "color" int

    // hue posterizer
    public static class HuePosterizer implements Filter {
        public int map(int pix) {
            return CameraLib.Pixel.hue(pix);             // return the Hue of the pixel
        }
    }

    // dominant color posterizer
    public static class DomColorPosterizer implements Filter {
        public int map(int pix) {
            return CameraLib.Pixel.dominantColor(pix);  // return the dominant color of the pixel
        }
    }

    // a simple wrapper around image data returned by the camera
    // that implements various pixel-scanning functions on the image
    public static class CameraImage {
        Size mSize;      // size of the camera that took this image
        byte[] mData;           // data from Camera preview in NV21 format
        Bitmap mBitmap;         // mData converted to RGB
        boolean mCameraRight=false;   // true if phone is oriented landscape with camera on right

        // make a CameraImage from Camera data (in NV21 format)
        // see http://www.fourcc.org/yuv.php#NV21 for descriptions of various formats
        public CameraImage(final byte[] imageData, Camera c) {
            mData = imageData;      // reference to (readonly) image data
            Camera.Parameters camParms = c.getParameters();
            Camera.Size size = camParms.getPictureSize();
            mBitmap = NV21toRGB.convert(mData, size.width, size.height);
            mSize = new Size(size.width, size.height);
        }

        // wrap an existing Bitmap in a CameraImage so we can use our search functions on it
        public CameraImage(Bitmap bm) {
            mBitmap = bm;
            mSize = new Size(mBitmap.getWidth(), mBitmap.getHeight());
            mData = null;
        }

        public Size cameraSize() {
            return mSize;
        }

        public void setCameraRight(boolean bRight) {
            mCameraRight = bRight;
        }

        public int dataSize() { return mData.length; }

        public String dataToString(int count) {
            String s = "";
            for (int i=0; i<count; i++) {
                int b = mData[i];
                if (b < 0)      // treat data as unsigned
                    b += 256;
                s += String.valueOf(b)+",";
            }
            return s;
        }

        // return the pixel at (x,y) as ARGB packed in an int
        // adjust for new orientation of camera, now assumed to be in landscape mode, lens facing front,
        // with the USB plug on the TOP (previously bottom) of the phone, so physical camera scanlines now go
        // x: right-to-left  y: bottom-to-top
        // but we will reverse that so our virtual scanlines still go (as before)
        // x: left-to-right  y: top-to-bottom
        public int getPixel(int x, int y) {
            return mCameraRight ? mBitmap.getPixel(mSize.width-1-x, mSize.height-1-y) : mBitmap.getPixel(x,y);
        }

        // return a string representation of the colors along the given scanline, using
        // the given posterizer to reduce int RGB to enumerated colors and then optionally
        // post-filtering those to further reduce the number of distinct colors recognized
        public String scanlineRep(int y, int bandWidth, Filter posterizer, Filter f) {
            Histogram hist = new Histogram(15);
            String sDom = "";               // string describing the line of pixels ito "dominant color"
            for (int x=0; x<cameraSize().width; x++) {
                int pix = getPixel(x, y);                    // get the pixel
                int domClr = posterizer.map(pix);            // get the posterized color of the pixel
                if (f != null)                               // if a mapping filter was provided
                    domClr = f.map(domClr);                    // use it to map values
                hist.add(domClr);                            // add a sample to Histogram for this band
                if (x%bandWidth == (bandWidth-1)) {
                    sDom += CameraLib.Pixel.colorName(hist.maxBin());  // add symbol string for most popular color in this band
                    hist.clear();                                // ... and restart the histogram for the next band
                }
            }
            return sDom;
        }

        // return a string representation of the dominant colors along the given scanline
        public String scanlineDomColor(int y, int bandWidth) {
            return scanlineDomColor(y, bandWidth, null);
        }
        public String scanlineDomColor(int y, int bandWidth, Filter f) {
            return scanlineRep(y, bandWidth, new DomColorPosterizer(), f);
        }

        // return a string representation of the hues along the given scanline
        public String scanlineHue(int y, int bandWidth) {
            return scanlineHue(y, bandWidth, null);
        }
        public String scanlineHue(int y, int bandWidth, Filter f) {
            return scanlineRep(y, bandWidth, new HuePosterizer(), f);
        }

        // return a string representation of the colors in each column-band of the image, using
        // the given posterizer to reduce int RGB to enumerated colors and then optionally
        // post-filtering those to further reduce the number of distinct colors recognized
        public String columnRep(int bandwidth, Filter posterizer, Filter f) {
            return columnRep(bandwidth, posterizer, f, 0);
        }

        public String columnRep(int bandWidth, Filter posterizer, Filter f, float minFrac) {
            Histogram hist = new Histogram(15);
            String sDom = "";               // string describing the line of pixels ito "dominant color"
            for (int x=0; x<cameraSize().width; x++) {
                for (int y=0; y<cameraSize().height; y++) {
                    int pix = getPixel(x, y);                        // get the pixel
                    int domClr = posterizer.map(pix);                // get the posterized color of the pixel
                    if (f != null)                                   // if a mapping filter was provided
                        domClr = f.map(domClr);                      // use it to map values
                    hist.add(domClr);                                // add the sample to Histogram for this band
                }
                if (x%bandWidth == (bandWidth-1)) {
                    int histmax = hist.maxBin();                     // get the index of the most popular entry
                    if (minFrac > 0 && hist.count(histmax) < minFrac * bandWidth * cameraSize().height)
                        sDom += "?";                                 // no color was dominant enough, so append a "?"
                    else
                        sDom += CameraLib.Pixel.colorName(histmax);  // add symbol string for most popular color in this band
                    hist.clear();                                    // ... and restart the histogram for the next band
                }
            }
            return sDom;
        }

        // return a string representation of the most popular dominant color in each column-band of the image
        public String columnDomColor(int bandWidth) {
            return columnDomColor(bandWidth, null);
        }
        public String columnDomColor(int bandWidth, Filter f) { return columnRep(bandWidth, new DomColorPosterizer(), f); }
        public String columnDomColor(int bandWidth, Filter f, float minFrac) { return columnRep(bandWidth, new DomColorPosterizer(), f, minFrac); }

        // return a string representation of the most popular hue in each column-band of the image
        public String columnHue(int bandWidth) {
            return columnHue(bandWidth, null);
        }
        public String columnHue(int bandWidth, Filter f) { return columnRep(bandWidth, new HuePosterizer(), f); }
        public String columnHue(int bandWidth, Filter f, float minFrac) { return columnRep(bandWidth, new HuePosterizer(), f, minFrac); }

        // return a string representation of the average color in each column-band of the image, using
        // the given posterizer to reduce int RGB to enumerated colors and then optionally
        // post-filtering those to further reduce the number of distinct colors recognized
        public String columnAvgRep(int bandWidth, Filter posterizer, Filter f) {
            String sDom = "";               // string describing the line of pixels ito "dominant color"
            int red = 0;
            int green = 0;
            int blue = 0;
            int npix = 0;
            for (int x=0; x<cameraSize().width; x++) {
                for (int y=0; y<cameraSize().height; y++) {
                    int pix = getPixel(x, y);                        // get the pixel
                    red += Color.red(pix);                           // accumulate the RGB contributions
                    green += Color.green(pix);
                    blue += Color.blue(pix);
                    npix++;
                }
                if (x%bandWidth == (bandWidth-1)) {
                    int avgColor = Color.rgb(red/npix, green/npix, blue/npix);
                    npix = 0; red = 0; green = 0; blue = 0;          // reset accumulators
                    int domClr = posterizer.map(avgColor);           // get the posterized color of the pixel
                    if (f != null)                                   // if a mapping filter was provided
                        domClr = f.map(domClr);                      // use it to map values
                    sDom += CameraLib.Pixel.colorName(domClr);       // add symbol string for the posterized average color in this band
                }
            }
            return sDom;
        }


        // convert relative (normalized) rectangle to physical one for this Bitmap
        public Rect relativeToPhysicalRect(RectF rF) {
            Rect r = new Rect(Math.round(rF.left*mSize.width), Math.round(rF.top*mSize.height), Math.round(rF.right*mSize.width), Math.round(rF.bottom*mSize.height));
            return r;
        }

        // return int representation of the dominant color in a rectangular region of the image, using
        // the given posterizer to reduce int RGB to enumerated colors and then optionally
        // post-filtering those to further reduce the number of distinct colors recognized
        public int rectRep(Rect r, Filter posterizer, Filter f) {
            Histogram hist = new Histogram(15);
            for (int x = r.left; x < r.right; x++) {
                for (int y = r.top; y < r.bottom; y++) {
                    int pix = getPixel(x, y);                       // get the pixel
                    int hue = posterizer.map(pix);                  // get the posterized color of the pixel
                    if (f != null)                                  // if a mapping filter was provided
                        hue = f.map(hue);                           // use it to map values
                    hist.add(hue);                                  // add the sample to Histogram for this rectangle
                }
            }
            return hist.maxBin();
        }

        // find the most common dominant color of pixels in a given rectangle of the image
        public int rectDomColor(Rect r) {
            return rectDomColor(r, null);
        }
        public int rectDomColor(Rect r, Filter f) {
            return rectRep(r, new DomColorPosterizer(), f);
        }
        public int rectDomColor(RectF rF) {
            return rectDomColor(rF, null);
        }
        public int rectDomColor(RectF rF, Filter f) {
            return rectRep(relativeToPhysicalRect(rF), new DomColorPosterizer(), f);
        }

        // find the most common hue of pixels in a given rectangle of the image
        public int rectHue(Rect r) {
            return rectHue(r, null);
        }
        public int rectHue(Rect r, Filter f) {
            return rectRep(r, new HuePosterizer(), f);
        }
        public int rectHue(RectF rF) {
            return rectHue(rF, null);
        }
        public int rectHue(RectF rF, Filter f) {
            return rectRep(relativeToPhysicalRect(rF), new HuePosterizer(), f);
        }

        // compute the centroid of pixels of a given color in the given rectangle of the image
        public Point centroid(Rect r, int color, Filter posterizer, Filter f) {
            int sumX = 0;
            int sumY = 0;
            int nPix = 0;
            for (int x = r.left; x < r.right; x++) {
                for (int y = r.top; y < r.bottom; y++) {
                    int pix = getPixel(x, y);                        // get the pixel
                    int hue = posterizer.map(pix);                   // get the posterized color of the pixel
                    if (f != null)                                   // if a mapping filter was provided
                        hue = f.map(hue);                            // use it to map values
                    if (color == hue) {          // if this pixel is what we're looking for ...
                        sumX += x;
                        sumY += y;
                        nPix++;
                    }
                }
            }
            if (nPix == 0)
                nPix++;
            int centX = sumX / nPix;
            int centY = sumY / nPix;
            return new Point(centX, centY);
        }

        // compute the centroid of pixels of a given dominant color in the given rectangle of the image
        public Point centroidDomColor(Rect r, int color) {
            return centroidDomColor(r, color, null);
        }
        public Point centroidDomColor(Rect r, int color, Filter f) {
            return centroid(r, color, new DomColorPosterizer(), f);
        }
        public Point centroidDomColor(RectF rF, int color) {
            return centroidDomColor(rF, color, null);
        }
        public Point centroidDomColor(RectF rF, int color, Filter f) {
            return centroid(relativeToPhysicalRect(rF), color, new DomColorPosterizer(), f);
        }

        // compute the centroid of pixels of a given hue in the given rectangle of the image
        public Point centroidHue(Rect r, int color) {
            return centroidHue(r, color, null);
        }
        public Point centroidHue(Rect r, int color, Filter f) {
            return centroid(r, color, new HuePosterizer(), f);
        }
        public Point centroidHue(RectF rF, int color) {
            return centroidHue(rF, color, null);
        }
        public Point centroidHue(RectF rF, int color, Filter f) {
            return centroid(relativeToPhysicalRect(rF), color, new HuePosterizer(), f);
        }

        public int count(Rect r, int color, Filter posterizer, Filter f) {
            int count = 0;
            for (int x = r.left; x < r.right; x++) {
                for (int y = r.top; y < r.bottom; y++) {
                    int pix = getPixel(x, y);                        // get the pixel
                    int hue = posterizer.map(pix);                   // get the posterized color of the pixel
                    if (f != null)                                   // if a mapping filter was provided
                        hue = f.map(hue);                            // use it to map values
                    if (color == hue) {          // if this pixel is what we're looking for ...
                        count++;                 // increment the count
                    }
                }
            }
            return count;
        }

        // compute the number of pixels of a given dominant color in the given rectangle of the image
        public int countDomColor(Rect r, int color) {
            return countDomColor(r, color, null);
        }
        public int countDomColor(Rect r, int color, Filter f) {
            return count(r, color, new DomColorPosterizer(), f);
        }
        public int countDomColor(RectF rF, int color) {
            return countDomColor(rF, color, null);
        }
        public int countDomColor(RectF rF, int color, Filter f) {
            return count(relativeToPhysicalRect(rF), color, new DomColorPosterizer(), f);
        }

        // compute the number of pixels of a given hue in the given rectangle of the image
        public int countHue(Rect r, int color) {
            return countHue(r, color, null);
        }
        public int countHue(Rect r, int color, Filter f) {
            return count(r, color, new HuePosterizer(), f);
        }
        public int countHue(RectF rF, int color) {
            return countHue(rF, color, null);
        }
        public int countHue(RectF rF, int color, Filter f) {
            return count(relativeToPhysicalRect(rF), color, new HuePosterizer(), f);
        }

        // return the given scanline as a Scanline object (an array of byte grayscale values)
        public Scanline scanlineGray(int y) {
            Scanline gray = new Scanline(cameraSize().width);
            for (int x=0; x<cameraSize().width; x++) {
                int pix = getPixel(x, y);
                int value = CameraLib.Pixel.value(pix);            // grayscale value 0:255
                gray.setPixel(x, value);
            }
            return gray;
        }
    }

    // class providing operations on a grayscale byte scanline
    public static class Scanline {
        int[] mData;           // array of byte grayscale pixels
        public Scanline(int width){
            mData = new int[width];
        }
        public Scanline(Scanline s) {
            mData = s.mData.clone();
        }
        public void setPixel(int x, int value) {
            mData[x] = value;
        }
        public int getPixel(int x) {
            return mData[x];
        }
        public void threshold(int value) {
            for (int x=0; x<mData.length; x++)
                mData[x] = (mData[x] > value) ? (byte)1 : (byte)0;
        }
        public int min() {
            int min = 255;
            for (int x=0; x<mData.length; x++)
                if (mData[x] < min)
                    min = mData[x];
            return min;
        }
        public int max() {
            int max = 0;
            for (int x=0; x<mData.length; x++)
                if (mData[x] > max)
                    max = mData[x];
            return max;
        }
        public int centroid() {
            // this call is destructive -- clone this object's data if you want to use it again after this call
            // threshold operation creates array of 0/1 - cut value is average of min and max intensities
            threshold((min()+max())/2);

            // compute center of mass of the array
            int c = 0;
            int n = 0;
            for (int x=0; x<mData.length; x++) {
                c += x * getPixel(x);     // position-weighted sum of non-zero entries
                n += getPixel(x);         // number of non-zero entries
            }
            return (n>0) ? c/n : -1;
        }
        public float fCentroid() {
            int c = centroid();
            return (c==-1) ? -1 : (float)c/(float)mData.length;
        }
    }

    // utility class that constructs a histogram of given data and reports various stats on it
    public static class Histogram {
        int[] mHist;
        public Histogram(int bins) {
            mHist = new int[bins];
            clear();
        }
        public void clear() {
            for (int i=0; i<mHist.length; i++)
                mHist[i] = 0;
        }
        public void add(int sample) {
            if (sample >= 0 && sample < mHist.length)
                mHist[sample]++;
        }
        public int maxBin() {
            int mx = 0;
            for (int i=1; i<mHist.length; i++)
                if (mHist[i] > mHist[mx])
                    mx = i;
            return mx;
        }
        public int count(int bin) {
            return mHist[bin];
        }
    }

    // utility class that wraps up all the data and logic needed to acquire image frames from the camera
    public static class CameraAcquireFrames {
        Camera mCamera;
        SurfaceTexture mDummyTexture;
        CameraImage mPreviewImage;
        int mFrameCount;
        boolean mNewFrame;

        public CameraAcquireFrames() {
            mCamera = null;
            mPreviewImage = null;
            mDummyTexture = null;
            mFrameCount = 0;
            mNewFrame = false;
        }

        Camera.PreviewCallback mPreviewCallback = new Camera.PreviewCallback() {
            public void onPreviewFrame(byte[] imageData, Camera camera) {
                // process the frame and save results in member variables
                // ...
                if (!mNewFrame) {
                    mPreviewImage = new CameraImage(imageData, camera);
                    mNewFrame = true;
                }
                mFrameCount++;
            }
        };

        public boolean init(int iCamSize) {
            try {
                if (mCamera == null)        // make sure we don't already have it ...
                    mCamera = Camera.open();
                Camera.Parameters parameters = mCamera.getParameters();
                List<Integer> previewFormats = parameters.getSupportedPreviewFormats();
                parameters.setPreviewFormat(ImageFormat.NV21);       // choices are NV21(17) and YV12(842094169)
                List<Camera.Size> previewSizes = parameters.getSupportedPictureSizes();
                int iSize = previewSizes.size()-iCamSize;  // use ith-smallest size (i>0): e.g. (1)160x120, (2)176x144, (3)320x240 entry
                parameters.setPreviewSize(previewSizes.get(iSize).width, previewSizes.get(iSize).height);
                parameters.setPictureSize(previewSizes.get(iSize).width, previewSizes.get(iSize).height);
                mCamera.setParameters(parameters);
                mDummyTexture = new SurfaceTexture(1); // make a target texture with an arbitrary texture id
                mCamera.setPreviewTexture(mDummyTexture);
                mCamera.setPreviewCallback(mPreviewCallback);
                mCamera.startPreview();
            }
            catch (Exception e) {
                if (mCamera != null)
                    mCamera.release();
                mCamera = null;
                return false;
            }
            return true;
        }

        public CameraImage loop() {
            if (mNewFrame) {
                mNewFrame = false;
                // start another frame acquisition
                try {
                    //mCamera.setPreviewCallback(mPreviewCallback);
                    //mCamera.startPreview();
                }
                catch (Exception e) {
                    return null;
                }
            }
            return mPreviewImage;       // may be null if no data ready yet
        }

        public int frameCount() {
            return mFrameCount;
        }

        public void stop() {
            if (mCamera != null) {          // guard against extra stop() calls ...
                mCamera.stopPreview();
                mCamera.setPreviewCallback(null);
                mCamera.release();
            }
            mCamera = null;         // delete the Camera object now
            mDummyTexture = null;
            mPreviewImage = null;
        }

    }

}
