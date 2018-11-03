package org.firstinspires.ftc.teamcode._Libs;

import android.graphics.Bitmap;
import android.graphics.RectF;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;

/**
 * Created by phanau on 9/14/18.
 */


/**
 * This library encapsulates the code needed by OpModes using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 *
 * Vuforia uses the phone's camera to inspect its surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * This library assumes a field configuration with four targets, one under each of four beacons,
 * mounted two each on two adjacent walls of the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * IMPORTANT: In order to use this Library, you need to obtain your own Vuforia license key as
 * is explained below.
 */

// we need to break into the "impl" class to get access to its close() function so we can
// really shut it down and get back control of the camera
class myRoverRuckusVuforiaLocalizerImpl extends VuforiaLocalizerImpl
{
    public myRoverRuckusVuforiaLocalizerImpl(VuforiaLocalizer.Parameters parameters)
    {
        super(parameters);
    }
    public void _close()  { close(); }
}


public class VuforiaLib_RoverRuckus implements HeadingSensor, LocationSensor {

    myRoverRuckusVuforiaLocalizerImpl vuforia;
    VuforiaTrackables trackables = null;
    List<VuforiaTrackable> allTrackables;
    OpenGLMatrix lastLocation = null;
    OpMode mOpMode;

    BlockingQueue<VuforiaLocalizer.CloseableFrame> mFrameQueue;
    VuforiaLocalizer.CloseableFrame mCF;

    public void init(OpMode opMode, String licenseKey) {

        // remember this so we can do telemetry output
        mOpMode = opMode;

        /**
         * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
         * the camera monitor feedback; if no camera monitor feedback is desired, use the parameterless
         * constructor instead. We also indicate which camera on the RC that we wish to use. For illustration
         * purposes here, we choose the back camera; for a competition robot, the front camera might
         * prove to be more convenient.
         *
         * Note that in addition to indicating which camera is in use, we also need to tell the system
         * the location of the phone on the robot; see phoneLocationOnRobot below.
         *
         * IMPORTANT: You should obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is valid and will function, but better to get your own.
         * Vuforia will not load without a valid license being provided. Vuforia 'Development' license
         * keys, which is what is needed here, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Valid Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string form of the key from the Vuforia web site
         * and paste it in to your code as the value of the 'vuforiaLicenseKey' field of the
         * {@link Parameters} instance with which you initialize Vuforia.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey =
                (licenseKey != null && licenseKey.length() > 0) ? licenseKey :
                        "ARf809H/////AAAAGRswBQwUCUJ5nqfgZxGbDEQ8oO7YP5GdnbReYr8ZHinqQ74OsP7UdOxNZJDmhaF2OeGD20jpSexpr2CcXGSGuHXNB2p9Z6zUNLDTfEggL+yg4ujefoqdkSpCqZf1medpwh3KXcK76FcfSJuqEudik2PC6kQW/cqJXnnHofVrrDTzJmWMnK3hlqTMjig81DEPMAHbRnA5wn7Eu0irnmqqboWyOlQ0xTF+P4LVuxaOUFlQC8zPqkr1Gvzvix45paWtyuLCnS9YDWMvI1jIM4giMrTRCT0lG8F+vkuKMiK647KJp9QIsFdWQ0ecQhau3ODNQ03pcTzprVN72b9VObpv6FNBpjGKRAcA59xlZiM2l6fc";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = new myRoverRuckusVuforiaLocalizerImpl(parameters); // ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data sets that for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
         * example "StonesAndChips", datasets can be found in in this project in the
         * documentation directory.
         */

        // the indices used below are determined by the order of the entries in the RoverRuckus.xml file
        trackables = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable craterTarget = trackables.get(2);
        craterTarget.setName("Craters");
        VuforiaTrackable footprintTarget = trackables.get(1);
        footprintTarget.setName("Footprint");
        VuforiaTrackable nebulaTarget = trackables.get(3);
        nebulaTarget.setName("Nebula");
        VuforiaTrackable roverTarget = trackables.get(0);
        roverTarget.setName("Rover");


        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(trackables);

        /**
         * Some folks use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="stones" size="247 173"/>
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         *
         * Currently, the dataset generated by PRH uses 1000 units for the width of the targets,
         * which are full-sheet A4 paper measuring 210x297mm, so one unit is 0.297mm.
         */
        float mmPerUnit = 0.297f;                            // units of targets created by PRH on 14 Sep 18
        float mmPerInch = 25.4f;
        float mmBotWidth = 18 * mmPerInch;                   // ... or whatever is right for your robot
        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
        float mmFTCTileSize = 24 * mmPerInch;                // size of field tiles
        float mmFTCTargetCenterZ = 6 * mmPerInch;            // height above field floor of centers of targets

        /**
         * In order for localization to work, we need to tell the system where each target we
         * wish to use for navigation resides on the field, and we need to specify where on the robot
         * the phone resides. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * For the most part, you don't need to understand the details of the math of how transformation
         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
         * <ol>
         *
         *     <li>You can put two transformations together to produce a third that combines the effect of
         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
         *     then the combined transformation matrix RT which does the rotation first and then the translation
         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
         *     <em>reverse</em> of the chronological order in which they applied.</li>
         *
         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
         *
         *     <li>If you want to break open the black box of a transformation matrix to understand
         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
         *
         * </ol>
         *
         * Initially the target is conceptually lying at the origin of the field's coordinate system
         * (the center of the field), facing up.
         *
         * In this configuration, the target's coordinate system aligns with that of the field.
         *
         * In a real situation we'd also account for the vertical (Z) offset of the target,
         * but for simplicity, we ignore that here; for a real robot, you'll want to fix that.
         *
         * To place the Craters Target on the wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it 180 around the field's Z axis to align it with the Front wall.
         * - Finally, we translate it in X and Y to its position on the wall.
         */
        craterTarget.setLocation(OpenGLMatrix
                .translation(0, -mmFTCFieldWidth/2, mmFTCTargetCenterZ)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 180, 0)));

        /*
         * To place the Footprint Target on the wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it -90 around the field's Z axis to align it with the Red wall.
         * - Finally, we translate it in X and Y to its position on the wall.
         */
        footprintTarget.setLocation(OpenGLMatrix
                .translation(mmFTCFieldWidth/2, 0, mmFTCTargetCenterZ)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, -90, 0)));

        /*
         * To place the Nebula Target on the wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Finally, we translate it in X and Y to its position on the Back wall.
         */
        nebulaTarget.setLocation(OpenGLMatrix
                .translation(0, mmFTCFieldWidth/2, mmFTCTargetCenterZ)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0)));

        /*
         * To place the Rover Target on the wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it 90 around the field's Z axis to align it with the Blue wall.
         * - Finally, we translate it in X and Y to its position on the wall.
         */
        roverTarget.setLocation(OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, 0, mmFTCTargetCenterZ)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0)));

        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone with the back camera facing the front of the robot (see our
         * choice of BACK camera above) and in landscape mode with the USB jack either up or down.
         * The starting alignment between the robot's and phone's axes is assumed to be phone
         * lying on the robot in portrait mode with the long axis (Y) pointing at the front of the robot.
         *
         * Rotations are right handed: i.e. positive rotations wrap each axis into the next (X-Y-Z-X)
         * in the direction of your fingers when your right thumb points down the remaining axis.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0, 0, mmFTCTargetCenterZ)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZX,
                        AngleUnit.DEGREES, 90, -90, 0));                // USB on top
        //       AngleUnit.DEGREES, -90, 90, 0));       // USB on bottom

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener) craterTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) footprintTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) nebulaTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) roverTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /**
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = phoneLocationOnRobot  maps   phone coords -> robot coords
         * P = tracker.getPose()     maps   image target coords -> phone coords
         * L = redTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()              maps   robot coords -> phone coords
         * P.inverted()              maps   phone coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        // get access to video frames so we can do other processing like looking for scene objects
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB format for the image
        vuforia.setFrameQueueCapacity(1);
        mFrameQueue = vuforia.getFrameQueue();
        mCF = null;
    }

    public void start()
    {
        /** Start tracking the data sets we care about. */
        trackables.activate();
    }

    public void loop(boolean bTelemetry)
    {
        lastLocation = null;    // reset each time so we can tell if we currently have any target visible

        for (VuforiaTrackable trackable : allTrackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            if (bTelemetry && ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible())
                mOpMode.telemetry.addData(trackable.getName(), "Visible");    //

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }

        /**
         * Provide feedback as to where the robot was last located (if we know).
         */
        if (bTelemetry) {
            if (haveLocation())
                mOpMode.telemetry.addData("Position:", formatPosition(lastLocation));
            else
                mOpMode.telemetry.addData("Position:", "Unknown");
            if (haveHeading())
                mOpMode.telemetry.addData("Orientation:", formatOrientation(lastLocation));
            else
                mOpMode.telemetry.addData("Orientation:", "Unknown");
        }
    }

    public void stop()
    {
        /** Stop tracking the data sets we care about. */
        trackables.deactivate();

        // close down Vuforia NOW so other code can use the camera
        vuforia._close();
    }

    // return lastLocation matrix (may be null)
    public OpenGLMatrix getLastLocation()
    {
        return lastLocation;
    }

    public VectorF getFieldPosition()
    {
        if (lastLocation != null)
            return lastLocation.getTranslation();
        else
            return null;
    }

    public Orientation getOrientation(AxesReference ref, AxesOrder order, AngleUnit unit)
    {
        if (lastLocation != null)
            return Orientation.getOrientation(lastLocation, ref, order, unit);
        else
            return null;
    }

    public Orientation getOrientation()
    {
        return this.getOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }

    public float getPitch()
    {
        return this.getOrientation().firstAngle;
    }

    public float getRoll()
    {
        return this.getOrientation().secondAngle;
    }

    public float getYaw()
    {
        return this.getOrientation().thirdAngle;    // -180 .. 0 .. +180
    }

    // implements HeadingSensor interface
    public float getHeading()
    {
        return getYaw();
    }
    public boolean haveHeading()
    {
        return (lastLocation != null);
    }

    // implements LocationSensor interface
    public VectorF getLocation() { return getFieldPosition(); }
    public boolean haveLocation() { return (lastLocation != null);}       // is there valid location data?

    /**
     * Some simple utilities that extract information from a transformation matrix
     * and format it in a form palatable to a human being.
     * For sanity's sake, display translations in inches rather than mm.
     */

    String formatPosition(OpenGLMatrix transformationMatrix) {
        //return transformationMatrix.formatAsTransform();
        VectorF translation = transformationMatrix.getTranslation();
        translation.multiply(1.0f/25.4f);       // convert from mm to inches
        return String.format("%s inches", translation.toString());
    }
    String formatOrientation(OpenGLMatrix transformationMatrix) {
        //return transformationMatrix.formatAsTransform();
        Orientation orientation = Orientation.getOrientation(transformationMatrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return String.format("%s", orientation.toString());
    }


    // get a bitmap from Vuforia - these are our own versions from 2017 code --
    // they don't use Vuforia convertFrameToBitmap function, which are essentially the same as these but don't do scaling or cropping as we do.

    // get entire image downsampled by an integer factor (e.g. 4 gives an image 1/4 the height and width of what the camera delivered)
    public Bitmap getBitmap(int sample) {
        return getBitmap(new RectF(0,0,1,1), sample);
    }

    // get entire image scaled by the given scale factor
    // (e.g. 4 gives an image 1/4 the height and width of what the camera delivered)
    public Bitmap getBitmap(float scale) {
        return getBitmap(new RectF(0,0,1,1), scale);
    }

    // get entire image scaled to the given width and height (result will be distorted if you don't get this right)
    public Bitmap getBitmap(int width, int height) {
        return getBitmap(new RectF(0,0,1,1), 0, width, height);
    }

    // get a cropped portion of the camera image downsampled by an integer factor
    // (e.g. sample=4 gives an image 1/4 the height and width of the cropped portion)
    public Bitmap getBitmap(RectF rect, int sample) {
        return getBitmap(rect, 1.0f/sample, 0, 0);
    }

    // get a cropped portion of the camera image scaled by a given factor
    // (e.g. scale=0.25 gives an image 1/4 the height and width of the cropped portion)
    public Bitmap getBitmap(RectF rect, float scale) {
        return getBitmap(rect, scale, 0, 0);
    }

    // get a cropped portion of the camera image scaled to the given width and height (result will be distorted if you don't get this right)
    public Bitmap getBitmap(RectF rect, int width, int height) {
        return getBitmap(rect, 0, width, height);
    }

    // this function handles all the various ways of describing the final output size
    // rect is the portion of the source image you want, normalized 0..1 in both axes
    private Bitmap getBitmap(RectF rect, float scale, int height, int width) {
        try {
            VuforiaLocalizer.CloseableFrame frame = mFrameQueue.take();
            int img = 0;
            for (; img < frame.getNumImages(); img++) {
                //telemetry.addData("Image format " + img, frame.getImage(img).getFormat());
                if (frame.getImage(img).getFormat() == PIXEL_FORMAT.RGB565) break;
            }

            if (img == frame.getNumImages()) throw new IllegalArgumentException("Incorrect format");

            // get the Image with the correct format and extract its data
            Image image = frame.getImage(img);
            ByteBuffer byteBuffer = image.getPixels();

            int h = image.getHeight();    // height of src data
            int w = image.getWidth();     // width of src data

            // convert the data to a Bitmap
            byteBuffer.rewind();
            Bitmap bitmap = Bitmap.createBitmap(w, h, Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(byteBuffer);

            // crop the Bitmap if requested
            Bitmap bitmapCropped = bitmap;
            int ch = Math.round(h*rect.height());    // height of cropped src data
            int cw = Math.round(w*rect.width());     // width of cropped src data
            if (ch < h || cw < w) {
                bitmapCropped = Bitmap.createBitmap(bitmap,
                        Math.max(0, Math.round(w*rect.left)), Math.max(0, Math.round(h*rect.top)), cw, ch);
                h = ch; w = cw;
            }

            // get requested output size in one of several ways
            int dstW = cw;
            int dstH = ch;
            if (width>0 && height>0) {
                dstW = width;
                dstH = height;
            }
            else if (scale > 0) {
                dstW = (int)(cw*scale);
                dstH = (int)(ch*scale);
            }

            // scale the (possibly cropped) result bitmap to the requested size
            Bitmap bitmapScaled = Bitmap.createScaledBitmap(bitmapCropped, dstW, dstH, true);

            frame.close();

            return bitmapScaled;
        }
        catch (Exception e) {}

        return null;
    }
}