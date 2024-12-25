// Port of CommonParameters.h
package org.firstinspires.ftc.teamcode.auto.xml;

import org.opencv.core.Rect;

// Parameters extracted from XML files.
public class VisionParameters {

    // In c++ these are structs but here we'll make all of the fields final.
    // Use public static nested classes for "packaging convenience".
    // See https://stackoverflow.com/questions/253492/static-nested-class-in-java-why
    // Create by Outer.Nested instance = new Outer.Nested();

    // From the image_parameters element of any XML file.
    public static class ImageParameters {
        public final String image_source;
        public final int resolution_width;
        public final int resolution_height;
        public final Rect image_roi;

        public ImageParameters(String pImageSource, int pWidth, int pHeight, Rect pImageROI) {
            image_source = pImageSource;
            resolution_width = pWidth;
            resolution_height = pHeight;
            image_roi = pImageROI;
        }
    }

    // From the gray_parameters element of any XML file.
    public static class GrayParameters {
        public final int median_target; // normalization target
        public final int threshold_low; // for binary thresholding

        public GrayParameters(int pTarget, int pLowThreshold) {
            median_target = pTarget;
            threshold_low = pLowThreshold;
        }
    }

    // From the hsv_parameters element of any XML file.
    public static class HSVParameters {
        public final String hue_name;
        public final int hue_low;
        public final int hue_high;
        public final int saturation_median_target; // normalization target
        public final int saturation_threshold_low; // for inRange thresholding
        public final int value_median_target; // normalization target
        public final int value_threshold_low; // for inRange thresholding

        public HSVParameters(String pHueName, int pHueLow, int pHueHigh,
                             int pSaturationMedianTarget, int pSaturationThreshholdLow,
                             int pValueMedianTarget, int pValueThreshholdLow) {
            hue_name = pHueName;
            hue_low = pHueLow;
            hue_high = pHueHigh;
            saturation_median_target = pSaturationMedianTarget;
            saturation_threshold_low = pSaturationThreshholdLow;
            value_median_target = pValueMedianTarget;
            value_threshold_low = pValueThreshholdLow;
        }
    }

    // From the lab_parameters element of any XML file.
    public static class LABParameters {
        public final double L_star_low;
        public final double L_star_high;
        public final double a_star_low;
        public final double a_star_high;
        public final double b_star_low;
        public final double b_star_high;

        public LABParameters(double pLStarLow, double pLStarHigh,
                             double pAStarLow, double pAStarHigh,
                             double pBStarLow, double pBStarHigh) {
            L_star_low = pLStarLow;
            L_star_high = pLStarHigh;
            a_star_low = pAStarLow;
            a_star_high = pAStarHigh;
            b_star_low = pBStarLow;
            b_star_high = pBStarHigh;
        }
    }

}
