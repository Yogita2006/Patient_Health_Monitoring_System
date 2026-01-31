# Patient Health Monitoring System

## Overview

The **Patient Health Monitoring System** is a healthcare-oriented embedded/IoT project developed to monitor essential physiological parameters of a patient in real time. The system focuses on continuous observation of vital signs such as body temperature and heart rate, enabling timely detection of abnormal conditions.

This project demonstrates the practical application of embedded systems and Internet of Things (IoT) concepts in the healthcare domain and aims to provide a basic yet extensible framework for remote patient monitoring.

## Objectives

* To continuously monitor vital health parameters of a patient
* To display real-time health data locally using a display module
* To enable remote monitoring through IoT platforms (optional)
* To assist caregivers and medical personnel in early detection of health issues

## Key Features

* Real-time monitoring of heart rate
* Real-time monitoring of body temperature
* Local display of sensor readings
* Optional integration with IoT dashboards for remote access
* Scalable architecture for adding additional health sensors

## Technologies Used

### Hardware Components

* Microcontroller (Arduino / ESP8266)
* Pulse Sensor
* Temperature Sensor (LM35 / DHT11)
* LCD Display (16×2)
* Power Supply Module

### Software & Tools

* Arduino IDE
* Embedded C / C++
* IoT Platforms (ThingSpeak / Blynk – optional)

## System Architecture

Sensors collect physiological data from the patient and send it to the microcontroller. The microcontroller processes the data and displays it on a local display. Optionally, the processed data can be transmitted to an IoT platform for remote monitoring and visualization.

```
Sensors → Microcontroller → LCD Display
                   ↓
            IoT Dashboard (Optional)
```

## Project Structure

```
Patient_Health_Monitoring_System/
├── src/          # Source code files
├── hardware/     # Circuit diagrams and hardware details
├── docs/         # Documentation and reports
├── README.md     # Project documentation
├── LICENSE       # License information
```

## Setup and Installation

### Hardware Setup

1. Connect the pulse sensor to the analog input pin of the microcontroller.
2. Connect the temperature sensor according to its specifications.
3. Interface the LCD display with the microcontroller.
4. Provide an appropriate power supply to the system.

### Software Setup

1. Install the Arduino IDE.
2. Install required libraries for sensors and display modules.
3. Open the main `.ino` file located in the `src` directory.
4. Select the correct board and COM port.
5. Upload the code to the microcontroller.

## Usage Instructions

1. Power on the system.
2. Attach the sensors to the patient correctly.
3. Observe real-time health parameters on the LCD display.
4. If IoT integration is enabled, monitor data remotely via the configured dashboard.

## Project Demonstration

This section is reserved for demonstrating the working of the project through images and videos. Visual documentation helps in better understanding of the system architecture, hardware setup, and real-time operation.

### Images

<img width="1919" height="943" alt="Screenshot 2025-11-28 182031" src="https://github.com/user-attachments/assets/9329c3c3-0f1a-4479-bf01-c616c615ca33" />

<img width="959" height="409" alt="Screenshot 2025-11-28 181751" src="https://github.com/user-attachments/assets/23259db9-2232-45c1-b5b8-b7222bffee27" />


![WhatsApp Image 2026-01-31 at 2 22 48 PM(1)](https://github.com/user-attachments/assets/44f6a128-e01e-42ad-85e2-24847dc34f47)


### Video Demonstration

A short video demonstrating the working of the system. This video include:

* Explanation of components
* Live monitoring of patient parameters
* Data visualization on display or IoT platform

You may upload the video to platforms like YouTube or Google Drive and provide the link below:

* Project Demo Video: *(Add link here)*

---

## Applications

* Remote patient health monitoring
* Elderly care systems
* Hospital monitoring systems
* Home healthcare solutions

## Future Enhancements

* Integration of additional sensors (SpO₂, ECG, Blood Pressure)
* Mobile or web-based monitoring application
* Data logging and analytics
* Alert and notification system for critical health conditions

## License

This project is licensed under the **MIT License**. Users are free to use, modify, and distribute the project with proper attribution.

---

*Developed as an academic and learning-oriented project in the field of embedded systems and IoT.*
