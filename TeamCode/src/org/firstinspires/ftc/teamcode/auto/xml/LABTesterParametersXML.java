package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
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
// needed test thresholding with the L*a*b* color space.
public class LABTesterParametersXML {
    public static final String TAG = LABTesterParametersXML.class.getSimpleName();
    private static final String GCP_FILE_NAME = "LABTesterParameters.xml";

    private final Document document;
    private final XPath xpath;

    public LABTesterParametersXML(String pXMLDir) {
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

    public LABTesterParameters getGoldCubeParameters() throws XPathExpressionException {
        XPathExpression expr;
        VisionParameters.GrayParameters grayParameters;
        VisionParameters.LABParameters labParameters;

        // Point to the first node.
        RobotLogCommon.d(TAG, "Parsing XML lab_tester_parameters");

        expr = xpath.compile("//lab_tester_parameters");
        Node lab_tester_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (lab_tester_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//lab_tester_parameters' not found");

        // Point to <gray_parameters>
        Node gray_node = lab_tester_parameters_node.getFirstChild();
        Node gray_parameters_node = XMLUtils.getNextElement(gray_node);
        if ((gray_parameters_node == null) || !gray_parameters_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' not found");

        grayParameters = ImageXML.parseGrayParameters(gray_parameters_node);

        // Point to <lab_parameters>
        Node lab_node = gray_parameters_node.getNextSibling();
        lab_node = XMLUtils.getNextElement(lab_node);
        if ((lab_node == null) || !lab_node.getNodeName().equals("lab_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'lab_parameters' not found");

        labParameters = ImageXML.parseLABParameters(lab_node);

        return new LABTesterParameters(grayParameters, labParameters);
    }

}

