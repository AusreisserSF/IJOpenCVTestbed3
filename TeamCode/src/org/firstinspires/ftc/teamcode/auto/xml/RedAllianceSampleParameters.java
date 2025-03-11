package org.firstinspires.ftc.teamcode.auto.xml;

import java.util.List;

// Input parameters for IntoTheDeep sample recognition.
public class RedAllianceSampleParameters {
    public final VisionParameters.GrayParameters redGrayParameters;
    public final SampleCriteria sampleCriteria;

    public RedAllianceSampleParameters(VisionParameters.GrayParameters pSampleGrayscaleParameters,
                                       SampleCriteria pSampleCriteria) {
        redGrayParameters = pSampleGrayscaleParameters;
        sampleCriteria = pSampleCriteria;
    }

    /*
    <criteria>
      <max_distance_from_target_sample_center>200</max_distance_from_target_sample_center>
      <min_sample_area>2000</min_sample_area>
      <max_sample_area>17000</max_sample_area>
      <min_sample_aspect_ratio>1.5</min_sample_aspect_ratio>
      <max_sample_aspect_ratio>3.0</max_sample_aspect_ratio>
    </criteria>
       */
    public static class SampleCriteria {
        public final double max_distance_from_target_sample_center;
        public final double min_sample_area;
        public final double max_sample_area;
        public final double min_sample_aspect_ratio;
        public final double max_sample_aspect_ratio;

        public SampleCriteria(double pMaxDistanceFromTargetSampleCenter,
                              double pMinSampleArea, double pMaxSampleArea,
                              double pMinSampleAspectRatio, double pMaxSampleAspectRatio) {
            max_distance_from_target_sample_center = pMaxDistanceFromTargetSampleCenter;
            min_sample_area = pMinSampleArea;
            max_sample_area = pMaxSampleArea;
            min_sample_aspect_ratio = pMinSampleAspectRatio;
            max_sample_aspect_ratio = pMaxSampleAspectRatio;
        }

    }

}