package org.firstinspires.ftc.teamcode.Blobs;

import org.firstinspires.ftc.teamcode.Blobs.blobdetect.Blob;
import org.firstinspires.ftc.teamcode.Blobs.blobdetect.BlobDetection;
import org.firstinspires.ftc.teamcode.Blobs.blobfilter.BlobFilter;
import org.firstinspires.ftc.teamcode.Blobs.blobtrack.MovingBlob;
import org.firstinspires.ftc.teamcode.Blobs.blobtrack.MovingBlobDetection;
import org.firstinspires.ftc.teamcode.Blobs.image.Color;
import org.firstinspires.ftc.teamcode.Blobs.image.Pixel;

import java.util.List;

public class PedestrianDetector {
    private BlobDetection blobDetection;
    private MovingBlobDetection movingBlobDetection;
    private BlobFilter blobFilter;

    public PedestrianDetector() {
        this(new BlobDetection(), new MovingBlobDetection(), new BlobFilter());
    }

    public PedestrianDetector(BlobDetection blobDetection, MovingBlobDetection movingBlobDetection, BlobFilter blobFilter) {
        this.blobDetection = blobDetection;
        this.movingBlobDetection = movingBlobDetection;
        this.blobFilter = blobFilter;
    }

    public List<MovingBlob> getAllBlobs(byte[] colors, int width) {
        int height = colors.length / width;
        Pixel[][] image = new Pixel[height][width];
        for (int i = 0; i < colors.length; i++) {
            int row = i / width;
            int col = i % width;
            image[row][col] = getPixel(colors[i]);
        }

        List<Blob> knownBlobs = blobDetection.getBlobs(image);
        return movingBlobDetection.getMovingBlobs(knownBlobs);
    }

    public List<MovingBlob> detect(byte[] colors, int width) {
        final List<MovingBlob> fmovingBlobs = blobFilter.filterMovingBlobs(getAllBlobs(colors, width));
        final List<MovingBlob> unifiedBlobs = movingBlobDetection.getUnifiedBlobs(fmovingBlobs);
        final List<MovingBlob> funifiedBlobs = blobFilter.filterUnifiedBlobs(unifiedBlobs);
        final List<MovingBlob> matchedUnifiedBlobs = movingBlobDetection.getFilteredUnifiedBlobs(funifiedBlobs);
        return blobFilter.filterFilteredUnifiedBlobs(matchedUnifiedBlobs);
    }

    public Pixel getPixel(byte b) {
        return new Pixel(Color.values()[b]);
    }
}
