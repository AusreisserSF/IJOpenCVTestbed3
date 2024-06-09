Stand-alone IntelliJ IDEA project for testing OpenCV image recognition.

This project is analogous to the CLion c++ project OpenCVTestbed3. OpenCV features may be implemented and tested first in OpenCVTestbed3 and then ported here *OR* they may be implemented and tested here in Java directly.

OpenCVTestbed3 and IJOpenCVTestbed3 have the same directory structure, command line parameters, and test case dispatching method based on a RobotAction.xml file.

From here OpenCV features may be ported to the IntelliJ project for a given FTC season, which has a naming
convention of "IJ" + game_title + "Vision", e.g. IJCenterStageVision, and from there to Android Studio. The IntelliJ project for a particular game has the same constants, enums, and directory structure as the Android Studio project for the same game.