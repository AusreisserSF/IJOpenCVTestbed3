package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.opencv.core.Mat;

import java.time.LocalDateTime;

public interface ImageProvider {

    // LocalDateTime requires minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
    Pair<Mat, LocalDateTime> getImage() throws InterruptedException;

}
