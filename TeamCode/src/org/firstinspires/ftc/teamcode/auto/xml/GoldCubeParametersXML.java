package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.auto.vision.GoldCubeParameters;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.firstinspires.ftc.teamcode.common.xml.ImageXML;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.*;
import java.io.File;
import java.io.IOException;

// Class whose job it is to read an XML file that contains all of the information
// needed for our OpenCV methods to recognize a gold cube during Autonomous.
public class GoldCubeParametersXML {
    public static final String TAG = GoldCubeParametersXML.class.getSimpleName();
    private static final String GCP_FILE_NAME = "GoldCubeParameters.xml";

    private final Document document;
    private final XPath xpath;

    public GoldCubeParametersXML(String pXMLDir) {
        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            document = dBuilder.parse(new File(pXMLDir + GCP_FILE_NAME));
            XPathFactory xpathFactory = XPathFactory.newInstance();
            xpath = xpathFactory.newXPath();

        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        } catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
        }
    }

    public GoldCubeParameters getGoldCubeParameters() throws XPathExpressionException {
        XPathExpression expr;
        VisionParameters.GrayParameters grayParameters;
        VisionParameters.HSVParameters hsvParameters;

        // Point to the first node.
        RobotLogCommon.d(TAG, "Parsing XML gold_cube_parameters");

        expr = xpath.compile("//gold_cube_parameters");
        Node gold_cube_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (gold_cube_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//gold_cube_parameters' not found");

        // Point to <gray_parameters>
        Node gray_node = gold_cube_parameters_node.getFirstChild();
        Node gray_parameters_node = XMLUtils.getNextElement(gray_node);
        if ((gray_parameters_node == null) || !gray_parameters_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' not found");

        grayParameters = ImageXML.parseGrayParameters(gray_parameters_node);

        // Point to <hsv_parameters>
        Node hsv_node = gray_parameters_node.getNextSibling();
        Node hsv_parameters_node = XMLUtils.getNextElement(hsv_node);
        if ((hsv_parameters_node == null) || !hsv_parameters_node.getNodeName().equals("hsv_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'hsv_parameters' not found");

        hsvParameters = ImageXML.parseHSVParameters(hsv_parameters_node);

        // Parse the size criteria for the bounding box.
        Node criteria_node = hsv_parameters_node.getNextSibling();
        criteria_node = XMLUtils.getNextElement(criteria_node);
        if (criteria_node == null)
            throw new AutonomousRobotException(TAG, "Element 'criteria' not found");

        // Parse the <min_bounding_box_area> element.
        Node min_area_node = criteria_node.getFirstChild();
        min_area_node = XMLUtils.getNextElement(min_area_node);
        if (min_area_node == null || !min_area_node.getNodeName().equals("min_bounding_box_area") ||
                min_area_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'min_bounding_box_area' not found or empty");

        String minAreaText = min_area_node.getTextContent();
        double minArea;
        try {
            minArea = Double.parseDouble(minAreaText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'min_bounding_box_area'");
        }

        // Parse the <max_bounding_box_area> element.
        Node max_area_node = min_area_node.getNextSibling();
        max_area_node = XMLUtils.getNextElement(max_area_node);
        if (max_area_node == null || !max_area_node.getNodeName().equals("max_bounding_box_area") ||
                max_area_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'max_bounding_box_area' not found or empty");

        String maxAreaText = max_area_node.getTextContent();
        double maxArea;
        try {
            maxArea = Double.parseDouble(maxAreaText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'max_bounding_box_area'");
        }

        GoldCubeParameters.BoundingBoxCriteria boundingBoxCriteria = new GoldCubeParameters.BoundingBoxCriteria(minArea, maxArea);

        return new GoldCubeParameters(grayParameters, hsvParameters, boundingBoxCriteria);
    }

}

