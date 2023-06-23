# Driver Drowsiness Prediction System with Machine Learning

The Driver Drowsiness Prediction System is a project that utilizes machine learning techniques to detect drowsiness in drivers and raise an alert to prevent potential accidents. This system is designed to reduce the number of fatalities caused by drowsy driving by providing timely warnings to drivers.

## Features

The system incorporates the following features:

1. Raspberry Pi Model 3 B: The Raspberry Pi Model 3 B is a credit card-sized single-board computer that serves as the hardware platform for the project. It provides the necessary computing power and connectivity options for implementing the drowsiness prediction system.

2. Haar Cascade Classifier: The Haar Cascade classifier is a machine learning-based approach used for object detection. In this system, the classifier is trained to detect facial landmarks and identify signs of drowsiness, such as closed eyes or yawning.

3. Python and OpenCV: Python is the programming language used for implementing the drowsiness prediction system. OpenCV (Open Source Computer Vision Library) is a popular library that provides tools and functions for computer vision tasks, including facial recognition and analysis.

4. Real-Time Monitoring: The system continuously monitors the driver's face in real-time using a camera. It analyzes the facial features and alerts the driver if signs of drowsiness are detected.

5. Alert Mechanism: When drowsiness is detected, the system triggers an alert to notify the driver and prevent potential accidents. This alert can take the form of an audible alarm, vibration, or visual warning.

## Dependencies

To run this Driver Drowsiness Prediction System, you will need the following dependencies:

- Raspberry Pi Model 3 B (or compatible)
- Python 3.x
- OpenCV
- NumPy

You can install OpenCV and NumPy on your Raspberry Pi by running the following command:

```
pip install opencv-python numpy
```

## Usage

1. Set up your Raspberry Pi Model 3 B and ensure that it is properly connected to a camera module.

2. Clone the repository or download the project files to your Raspberry Pi.

3. Install the required dependencies mentioned above.

4. Open a terminal on the Raspberry Pi and navigate to the project directory.

5. Run the following command to start the drowsiness prediction system:

```
python drowsiness_yawn.py
```

6. The system will access the camera module and begin monitoring the driver's face in real-time.

7. If signs of drowsiness, such as closed eyes or yawning, are detected, the system will trigger an alert to warn the driver.

## How it Reduces Fatalities

Drowsy driving is a major cause of accidents, often resulting in fatalities. The Driver Drowsiness Prediction System addresses this issue by providing timely warnings to drivers when signs of drowsiness are detected. By alerting drivers and bringing their attention back to the road, the system helps prevent accidents caused by drowsy driving. This can significantly reduce the number of fatalities and injuries on the road.

## Future Scope

There are several potential areas for future enhancement and expansion of the Driver Drowsiness Prediction System:

1. Integration with Vehicle Control Systems: The system can be further developed to integrate with vehicle control systems. For example, it could automatically activate safety features like lane departure warning or adaptive cruise control when drowsiness is detected.

2. Improved Drowsiness Detection: The current system primarily relies on facial landmarks for drowsiness detection. Future improvements could include additional features, such as eye tracking or head movement analysis, to enhance the accuracy of drowsiness prediction.

3. Data Logging and Analysis: Incorporating data logging and analysis capabilities into the system can help gather insights into driver behavior, drowsiness patterns, and potential risk factors
