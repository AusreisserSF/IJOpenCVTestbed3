// Port of XMLCommon.cpp
package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.opencv.core.Rect;
import org.w3c.dom.Node;

public class ImageXML {

    public static final String TAG = ImageXML.class.getSimpleName();

    // Parse the XML elements that describe the image to be analyzed.
    /*
    <image_parameters>
      <!-- IntelliJ projects: The image_source element contains a file name ending
           in .png or .jpg. -->
      <!-- Android Studio projects: the image_source element contains a string that
           is used as the first part of the file name when an image from a camera
           is written out to a file.
      <image_source></image_source>
      <resolution>
	    <width></width>
	    <height></height>
      </resolution>
      <image_roi>
	    <x></x>
	    <y></y>
	    <width></width>
	    <height></height>
      </image_roi>
    </image_parameters>
    */
    // Parse the children of the <image_parameters> element in the XML file.
    public static VisionParameters.ImageParameters parseImageParameters(Node pImageParametersNode) {
        String image_source;
        int resolution_width;
        int resolution_height;
        Rect image_roi;

        //RobotLogCommon.d(TAG, "Parsing XML image_parameters");

        if ((pImageParametersNode == null) || !pImageParametersNode.getNodeName().equals("image_parameters"))
            throw new AutonomousRobotException(TAG, "Missing required <image_parameters> element");

        Node image_source_node = pImageParametersNode.getFirstChild();
        image_source_node = XMLUtils.getNextElement(image_source_node);
        if ((image_source_node == null) || !image_source_node.getNodeName().equals("image_source") || image_source_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'image_source' not found");

        image_source = image_source_node.getTextContent();

	    /*
	    <resolution>
		  <width></width>
		  <height></height>
	    </resolution>
	    */
        Node resolution_node = image_source_node.getNextSibling();
        resolution_node = XMLUtils.getNextElement(resolution_node);
        if ((resolution_node == null) || !resolution_node.getNodeName().equals("resolution"))
            throw new AutonomousRobotException(TAG, "Element 'resolution' not found");

        // Get the two children of the resolution node: width and height
        Node resolution_width_node = resolution_node.getFirstChild();
        resolution_width_node = XMLUtils.getNextElement(resolution_width_node);
        if ((resolution_width_node == null) || !resolution_width_node.getNodeName().equals("width") || resolution_width_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'resolution/width' missing or empty");

        try {
            resolution_width = Integer.parseInt(resolution_width_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'resolution/width'");
        }

        Node resolution_height_node = resolution_width_node.getNextSibling();
        resolution_height_node = XMLUtils.getNextElement(resolution_height_node);
        if ((resolution_height_node == null) || !resolution_height_node.getNodeName().equals("height") || resolution_height_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'resolution/height' missing or empty");

        try {
            resolution_height = Integer.parseInt(resolution_height_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'resolution/height'");
        }

        // Parse the region of interest parameters.
        Node image_roi_node = resolution_node.getNextSibling();
        image_roi_node = XMLUtils.getNextElement(image_roi_node);
        if ((image_roi_node == null) || !image_roi_node.getNodeName().equals("image_roi"))
            throw new AutonomousRobotException(TAG, "Element 'image_roi' not found");

        image_roi = parseROI(image_roi_node);

        return new VisionParameters.ImageParameters(image_source, resolution_width, resolution_height, image_roi);
    }

    // Parse any element that contains the 4 ROI children.
    /*
    <!-- any node -->
	  <x></x>
	  <y></y>
	  <width></width>
	  <height></height>
    */
    private static Rect parseROI(Node pROINode) {
        int roiX, roiY, roiWidth, roiHeight;

        Node roi_x_node = pROINode.getFirstChild();
        roi_x_node = XMLUtils.getNextElement(roi_x_node);
        if ((roi_x_node == null) || !roi_x_node.getNodeName().equals("x") || roi_x_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'x' missing or empty");

        try {
            roiX = Integer.parseInt(roi_x_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'x'");
        }

        Node roi_y_node = roi_x_node.getNextSibling();
        roi_y_node = XMLUtils.getNextElement(roi_y_node);
        if ((roi_y_node == null) || !roi_y_node.getNodeName().equals("y") || roi_y_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'y' missing or empty");

        try {
            roiY = Integer.parseInt(roi_y_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'y'");
        }

        // Get the width and height elements
        Node roi_width_node = roi_y_node.getNextSibling();
        roi_width_node = XMLUtils.getNextElement(roi_width_node);
        if ((roi_width_node == null) || !roi_width_node.getNodeName().equals("width") || roi_width_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'width' missing or empty");

        try {
            roiWidth = Integer.parseInt(roi_width_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'width'");
        }

        Node roi_height_node = roi_width_node.getNextSibling();
        roi_height_node = XMLUtils.getNextElement(roi_height_node);
        if ((roi_height_node == null) || !roi_height_node.getNodeName().equals("height") || roi_height_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'height' missing or empty");

        try {
            roiHeight = Integer.parseInt(roi_height_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'height'");
        }

        return new Rect(roiX, roiY, roiWidth, roiHeight);
    }

    // Parse the children of the <gray_parameters> element in the XML file.
    public static VisionParameters.GrayParameters parseGrayParameters(Node pGrayNode) {
        int median_target;
        int threshold_low;

        //RobotLogCommon.d(TAG, "Parsing XML gray_parameters");

        if ((pGrayNode == null) || !pGrayNode.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Missing required <gray_parameters> element");

        Node gray_median_target_node = pGrayNode.getFirstChild();
        gray_median_target_node = XMLUtils.getNextElement(gray_median_target_node);
        if ((gray_median_target_node == null) || !gray_median_target_node.getNodeName().equals("median_target") || gray_median_target_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'median_target' missing or empty");

        try {
            median_target = Integer.parseInt(gray_median_target_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'median_target'");
        }

        Node gray_threshold_node = gray_median_target_node.getNextSibling();
        gray_threshold_node = XMLUtils.getNextElement(gray_threshold_node);
        if ((gray_threshold_node == null) || !gray_threshold_node.getNodeName().equals("threshold_low") || gray_threshold_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'threshold_low' missing or empty");

        try {
            threshold_low = Integer.parseInt(gray_threshold_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'threshold_low'");
        }

        return new VisionParameters.GrayParameters(median_target, threshold_low);
    }

    // Parse the children of the <hsv_parameters> element in the XML file.
    /*
    <hsv_parameters>
      <hue_name>gold</hue_name>
      <low_hue>10</low_hue>
      <high_hue>30</high_hue>
	  <saturation_median_target>200</saturation_median_target>
	  <saturation_threshold_low>165</saturation_threshold_low>
	  <value_median_target>200</value_target>
      <value_threshold_low>180</value_threshold_low>
    </hsv_parameters>
    */
    // At this point pHSVNode points to the <hsv_parameters> element.
    public static VisionParameters.HSVParameters parseHSVParameters(Node pHSVNode) {
        String hue_name;
        int hue_low;
        int hue_high;
        int saturation_median_target;
        int saturation_threshold_low;
        int value_median_target;
        int value_threshold_low;

        //RobotLogCommon.d(TAG, "Parsing XML hsv_parameters");

        if ((pHSVNode == null) || !pHSVNode.getNodeName().equals("hsv_parameters"))
            throw new AutonomousRobotException(TAG, "Missing required <hsv_parameters> element");

        Node hue_name_node = pHSVNode.getFirstChild();
        hue_name_node = XMLUtils.getNextElement(hue_name_node);
        if ((hue_name_node == null) || !hue_name_node.getNodeName().equals("hue_name") || hue_name_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'hue_name' missing or empty");

        hue_name = hue_name_node.getTextContent();
        RobotLogCommon.d(TAG, "Parsed XML hsv_parameters; hue name = " + hue_name);

        Node hue_low_node = hue_name_node.getNextSibling();
        hue_low_node = XMLUtils.getNextElement(hue_low_node);
        if ((hue_low_node == null) || !hue_low_node.getNodeName().equals("hue_low") || hue_low_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'hue_low' missing or empty");

        try {
            hue_low = Integer.parseInt(hue_low_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'hue_low'");
        }

        Node hue_high_node = hue_low_node.getNextSibling();
        hue_high_node = XMLUtils.getNextElement(hue_high_node);
        if ((hue_high_node == null) || !hue_high_node.getNodeName().equals("hue_high") || hue_high_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'hue_high' missing or empty");
        try {
            hue_high = Integer.parseInt(hue_high_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'hue_high'");
        }

        // <saturation_median_target>
        Node saturation_median_target_node = hue_high_node.getNextSibling();
        saturation_median_target_node = XMLUtils.getNextElement(saturation_median_target_node);
        if ((saturation_median_target_node == null) || !saturation_median_target_node.getNodeName().equals("saturation_median_target") || saturation_median_target_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'saturation_median_target' missing or empty");
        try {
            saturation_median_target = Integer.parseInt(saturation_median_target_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'saturation_median_target'");
        }

        // <saturation_threshold_low>
        Node saturation_threshold_low_node = saturation_median_target_node.getNextSibling();
        saturation_threshold_low_node = XMLUtils.getNextElement(saturation_threshold_low_node);
        if ((saturation_threshold_low_node == null) || !saturation_threshold_low_node.getNodeName().equals("saturation_threshold_low") || saturation_threshold_low_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'saturation_threshold_low' missing or empty");
        try {
            saturation_threshold_low = Integer.parseInt(saturation_threshold_low_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'saturation_threshold_low'");
        }

        // <value_median_target>
        Node value_target_node = saturation_threshold_low_node.getNextSibling();
        value_target_node = XMLUtils.getNextElement(value_target_node);
        if ((value_target_node == null) || !value_target_node.getNodeName().equals("value_median_target") || value_target_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'value_median_target' missing or empty");

        try {
            value_median_target = Integer.parseInt(value_target_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'value_median_target'");
        }

        // <value_threshold_low>
        Node value_threshold_low_node = value_target_node.getNextSibling();
        value_threshold_low_node = XMLUtils.getNextElement(value_threshold_low_node);
        if ((value_threshold_low_node == null) || !value_threshold_low_node.getNodeName().equals("value_threshold_low") || value_threshold_low_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'value_threshold_low' missing or empty");

        try {
            value_threshold_low = Integer.parseInt(value_threshold_low_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'value_threshold_low'");
        }

        return new VisionParameters.HSVParameters(hue_name, hue_low, hue_high,
                saturation_median_target, saturation_threshold_low,
                value_median_target, value_threshold_low);
    }

    // Parse the children of the <lab_parameters> element in the XML file.
    /*
    <lab_parameters>
      <l_star_low>25.0</l_star_low>
      <l_star_high>127.5</l_star_high>
      <a_star_low>178.0</a_star_low>
      <a_star_high>203.0</a_star_high>
      <b_star_low>153.0</b_star_low>
      <b_star_high>188.0</b_star_high>
    </lab_parameters>
    */
    // At this point pLABNode points to the <hsv_parameters> element.
    public static VisionParameters.LABParameters parseLABParameters(Node pLABNode) {
        double L_star_low;
        double L_star_high;
        double a_star_low;
        double a_star_high;
        double b_star_low;
        double b_star_high;

        //RobotLogCommon.d(TAG, "Parsing XML lab_parameters");

        if ((pLABNode == null) || !pLABNode.getNodeName().equals("lab_parameters"))
            throw new AutonomousRobotException(TAG, "Missing required <lab_parameters> element");

        Node l_low_node = pLABNode.getFirstChild();
        l_low_node = XMLUtils.getNextElement(l_low_node);
        if ((l_low_node == null) || !l_low_node.getNodeName().equals("L_star_low") || l_low_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'L_star_low' missing or empty");

        try {
            L_star_low = Double.parseDouble(l_low_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'L_star_low'");
        }

        Node l_high_node = l_low_node.getNextSibling();
        l_high_node = XMLUtils.getNextElement(l_high_node);
        if ((l_high_node == null) || !l_high_node.getNodeName().equals("L_star_high") || l_high_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'L_star_high' missing or empty");
        try {
            L_star_high = Double.parseDouble(l_high_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'L_star_high'");
        }

        Node a_low_node = l_high_node.getNextSibling();
        a_low_node = XMLUtils.getNextElement(a_low_node);
        if ((a_low_node == null) || !a_low_node.getNodeName().equals("a_star_low") || a_low_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'a_star_low' missing or empty");
        try {
            a_star_low = Double.parseDouble(a_low_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'a_star_low'");
        }

        Node a_high_node = a_low_node.getNextSibling();
        a_high_node = XMLUtils.getNextElement(a_high_node);
        if ((a_high_node == null) || !a_high_node.getNodeName().equals("a_star_high") || a_high_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'a_star_high' missing or empty");
        try {
            a_star_high = Double.parseDouble(a_high_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'a_star_high'");
        }

        Node b_low_node = a_high_node.getNextSibling();
        b_low_node = XMLUtils.getNextElement(b_low_node);
        if ((b_low_node == null) || !b_low_node.getNodeName().equals("b_star_low") || b_low_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'b_star_low' missing or empty");
        try {
            b_star_low = Double.parseDouble(b_low_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'b_star_low'");
        }

        Node b_high_node = b_low_node.getNextSibling();
        b_high_node = XMLUtils.getNextElement(b_high_node);
        if ((b_high_node == null) || !b_high_node.getNodeName().equals("b_star_high") || b_high_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'b_star_high' missing or empty");
        try {
            b_star_high = Double.parseDouble(b_high_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'b_star_high'");
        }

        return new VisionParameters.LABParameters(L_star_low, L_star_high,
                a_star_low, a_star_high, b_star_low, b_star_high);
    }
}
