package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.*;
import org.opencv.core.Mat;

import java.time.LocalDateTime;
import java.util.Date;

public class FileImage implements ImageProvider {

    private final String pathToImageFile;
    private final ImageUtils imageUtils = new ImageUtils();

    public FileImage(String pPathToImageFile) {
        pathToImageFile = pPathToImageFile;
    }

    @Override
    // LocalDateTime requires minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
    public Pair<Mat, LocalDateTime> getImage() {
        Mat bgrMat = imageUtils.loadImage(pathToImageFile);    // LocalDateTime requires minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        if (bgrMat == null) // no such file
            return null; // let the caller decide what to do
        return Pair.create(bgrMat, LocalDateTime.now());
    }

}

