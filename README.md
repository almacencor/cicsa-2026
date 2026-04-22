
<img width="842" height="468" alt="image" src="https://github.com/user-attachments/assets/f008d91d-e2a0-4c5f-97a4-c56cfcc7fe91" />

This is the official repository of the Team CICSA for the National final of the WRO2026 season .


## Follow us!

| Facebook | YouTube | Instagram |
|---------------|---------------|----------------|
| [![Facebook](other/facebook.jpg)](https://www.facebook.com/share/1A1hz6zQSn/) | [![YouTube](other/youtube.png)](https://www.youtube.com/@CICSA_Academia) | [![Instagram](other/instagram.jpg)](https://share.google/ZzGMFZOurGsjhWN2D) |


## Contents.
**Folders**
- [📁 Models](./models/)
- [📁 Other](./other/)
- [📁 Schemes](./schemes/)
- [📁 Src](./src/)
- [📁 T-photos](./t-photos/)
- [📁 V-photos](./v-photos/)
- [📁 Video](./video/video.md)

**Index**

- [The team](#The-team)
- [The Challenge.](#The-Challenge)
- [Robot Overview](#Robot-Overview)
- [Robot construction guide.](#Robot-construction-guide)
- [Engineering materials](#Engineering-materials)
- [Mobility Management](#Mobility-Management)
- [Power and Sensor Management](#Power-and-Sensor-Management)
- [Obstacle management](#Obstacle-management)
- [Performance videos](#Performance-videos)
- [Digital Engineering Logbook](https://github.com/CICSA-NET/WRO2025-CICSA/blob/main/other/README.md)
- [Final Remarks and future work](#Final-remarks-and-future-work)
- [References](#References)

___
## The team.
====

CICSA WRO 2026 - Future Engineers


Coach **Sergio Iván Hernández Ruiz**

Coach Sergio Iván provides the technical guidance and leadership required to keep Team CICSA on track. With extensive experience in robotics, he helps us navigate complex challenges, refine our designs, and develop solutions that work in competitive and real-world settings.

Sergio Iván has been the director of the CICSA robotics academy since 2015 and has participated in events organized by the WRO since 2019.
***
**Members:**

**Gildardo Garcia**, at 16 years old, he studies in the Centro de Estudios Tecnológicos Industrial y de Servicios 128. He has participate in the 2019 Regional in Guadalajara in the primary category, where he obtained 2nd place.
***
**Sergio Amid Hernández Gónzalez**, at 19 years old, he studies Software Engineering at Kuepa University. He has participated in numerous competitions, including: the 2019 WRO Regional in Guadalajara, Jalisco, México, where he obtained 1st place. In 2024, in the city of Mexicali, Baja California, México, they obtained 1st place in the preparatory category. He was part of the Mexican robotics team that represented Mexico at the ChampionShip in Italy in 2024.
***
**Diego Pereida Arochi**, At 17 years old, he is a student at the Nogales high school. He has been in numerous competitions, including the: FIRST robotics competition in the United states and in Mexico has participated thourgh the years in the WRO robomission challange achieving 1st place in WRO 2025 Mexicali and along side Sergio Amid made it to the WRO Italy 2024 open world championship to respresent Mexico.

___




[📁 T-photos](./t-photos/)


[Menu](#Contents)

___

## The Challenge.
====

The WRO Future Engineers challenge pushes students to create fully autonomous self driving vehicles. Each robot must:

- Navigate a dynamically randomized track.
- Detect and avoid colored obstacles (green/red blocks).
- Execute a parallel parking maneuver.

Scoring is based on:
- Performance on track.
- Obstacle handling.
- Documentation quality.
- Innovation and engineering rigor.

Objective:
Design and build a mobile robot capable of autonomous navigation using: 
* Image recognition for object detection.
* PID control to maintain a safe distance from obstacles using ultrasonic sensors.
* Communication between Raspberry Pi 5 and ESP32 to divide tasks between heavy processing (vision) and real-time control (motors and sensors).

**Open challenge.**

<img width="817" height="459" alt="image" src="https://github.com/user-attachments/assets/ff3d98e8-99d8-4e3b-b802-f49cc1a7c85c" />



**Obstacle challenge.**

<img width="963" height="552" alt="image" src="https://github.com/user-attachments/assets/ebe243d9-7ccf-4ea6-b62b-a09898a56666" />


For more indo visit: [WRO Official Site](https://wro-association.org/)


[Menu](#Contents)
___

## Robot Overview.
====

| Front | Back | Top |
|---------------|---------------|----------------|
| 
| Bottom | Left | Right |
|---------------|---------------|----------------|
| |


| Side Dimensions | Front Dimensions | Robot Weight| 
|---------------|---------------|----------------|
|  |


| Evolution of the robot |
|---------------|
| ![] |


[Menu](#Contents)
___

## Robot construction guide.
====

**Assembly Guide.**

This guide provides step-by-step instructions for assembling the robot chassis. It includes mechanical installation, electrical wiring, and safety notes for robotics projects using ESP-32, Raspberry Pi 4, LIDAR, camera and sensors.

---

**Component List.**

| Component                   | Quantity | Description                     |
|----------------------------|----------|---------------------------------|
|LIDAR A1M8                  | 1        |LIDAR system for navigation      |
|URM37 V5                    | 2        |Ultrasonic sensor                |
|N20 DC motor 12V 30RPM      | 2        |Motor for drive system           |
|Wheels for 1/18 TRX4M       | 4        |Includes assembly                |
|Steering Shaft 4mm to 3mm   | 1        |Wheel connection for torque      |
|ESP32                       | 1        |Motor system controller          |
|Raspberry Pi 4 Modelo B     | 1        |Main system controller           |
|Standard Front Rear Axles   | 2        |Steering system                  |
|11kg Servo Moto             | 1        |Steering system                  |


---

**Pre-Installation Checks.**

**Servo Motor.**
- Connect and calibrate to **90°** before installation.
- Use test code or compatible software.

**DC Motor.**
- Power directly to verify rotation.
- **Red dot** indicates positive terminal.

---

**Wheel Assembly Steps.**

---

**Servo & Steering Assembly.**


---

**Chassis Mounting.**

For more information regarding 3D printed parts: [📁 Models](./models/)

---

**Electrical Wiring.**

**Servo Motor.**

| Wire Color | Function |
|------------|----------|
| Brown      | GND      |
| Red        | VCC      |
| Yellow     | Signal   |

**Power Switch.**

- Connect **in series** with the positive (red) wire.
- Enables safe power control.

---

**Safety Notes.**

- Do not operate in humid environments.
- Avoid reverse polarity.
- Prevent short circuits.
- Double-check connections before powering on.

---

**Board Compatibility.**

Mounting holes support:

- ESP-32 base
- Camera base
- Raspberry Pi 4 base

---

Original PDF: [Elecrow 4WD Car Installation Instructions](https://www.elecrow.com/download/4wd_CAR_Install_instructions.pdf)

---




[Menu](#Contents)
___



## Engineering materials.
====

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2025.

| Quantity | Component                        |Link                                                                            |
|----------|----------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1        | Raspberry Pi 4 (4GB)             |[Amazon](https://a.co/d/084kiOZ5)                                                                                                                                                                                                                                                                                                                                     |
| 1        | ESP-WROOM-32                     |[Amazon](https://www.amazon.com/ESP-WROOM-32-Development-Microcontroller-Integrated-Compatible/dp/B08D5ZD528/ref=sr_1_3?crid=4UOP8VE8GTTS&dib=eyJ2IjoiMSJ9.XBINg-sjhfF_gUtnMiKGjjEQQzaaOnS0BOX5B4WtqfKi_hDEOXP7QQezpOTp8e-r5VfFo411y5_IeCcg7DdMeiEcUBSTnC23mXHz1NRbrXP4JVyQShmo1b-y4KGfxRlXgSbH3VJTSz55fBe_QqxKedXkggHhw1Qs7-1O_ErcV1o19Py9to1BKZQxieZLm_8y4zb-6LtXq2DwHFZG0EPFL_AJ_bL6bAtaLzdAk6A-5WPFche0M6tcFhuRjMkOpAN2zYFYcGYeWiFqfLdcRUPoiY2-8jpuuo8vkBUfver4Gk4.mo41uc2rmPqL0XJ5jHK3kmN_x8PFmvrfhMfZ2HkptCM&dib_tag=se&keywords=Esp-Wroom-32&qid=1753387413&s=electronics&sprefix=esp-wroom-32%2Celectronics%2C294&sr=1-3&th=1)                                                                                                                                                                              |
| 1        | Camara Freenove 8MP              |[Amazon](https://www.amazon.com/Freenove-8MP-Raspberry-Adjustable-Viewing/dp/B0BZYPBS17/ref=sr_1_1_sspa?crid=2ZNF0R4SWCQWT&dib=eyJ2IjoiMSJ9.f4-pR4WTqQbw0vioJRPdz0qtF5MiHJl-MhiG_Yo9dxwZXPo4CsqP2Iiyt_ae639ZQ68gVBs98taIG5zMf0PXKR6IMyUoCDn_jD7_hGgcOXlWreSjEqpqQpZTBdGct0sKvzvACoLi4qHOlHhM0e-xMC_308CRUrspoesSuOSSsMM.nVcijt2RJWCE17iYMeWy3e3SG6eqijPvlEt37sYoc44&dib_tag=se&keywords=Freenove%2B8MP%2Bcamera&qid=1753387520&s=electronics&sprefix=freenove%2B8mp%2Bcamera%2Celectronics%2C285&sr=1-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1)                                                                                                                                                                              |
| 1        | Driver de motor L298N            |[Amazon](https://www.amazon.com/JTAREA-Stepper-Driver-H-Bridge-Mega2560/dp/B0D2RJV2VX/ref=sr_1_2_sspa?crid=3C1LW4P6C7MR1&dib=eyJ2IjoiMSJ9.hK2FjV8Ukp8CCyVTI1seMoTh-okctAQYG9wv8_2wWQicu3tQcZGLcWqQEfZJZiEpCHtA15XAfvFsTRtoXRJ6wEG8xSTcdPfjtlfmyDqol-3C38gVuPnv3FlabrIutV5q3wzdwbimQaUZF726_CESUC5PDSaG-sz7VeI6AP5hrkIpA2wr_m-ZQoxYgSgCYoBXOSU32LgaoI1_3YNdT_o4-caslfve1JPRLD6TIg68hNFwg8Zbz9FSaIh7d_UwKp6OgV7lm5ut-c0lOlitzyWm5AU5nTyhq4AwX1OCGBCl33Y.P4ZzI9RZbRt-hM7EfbWZSR7GTTefFCgTi5MY6UKLEI0&dib_tag=se&keywords=L298N%2Bmotor%2Bdriver&qid=1753387552&s=electronics&sprefix=l298n%2Bmotor%2Bdriver%2Celectronics%2C173&sr=1-2-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1)                                                                        

Estimated total cost ==> **$ ????? US dollars**.


|    Component         |   Function / Description                          |
|----------------------|---------------------------------------------------|
| Raspberry Pi 4       | Image processing and navigation logic             |
| ESP32                | Real-time servo control with PID algorithms and read sensors          |
| Camera               | Captures images for computer vision               |
| ARM37V5 (x3)         | Distance measurement for obstacle avoidance       |
| L298N Motor Driver   | Controls DC motors                                |
| N20 DC Motor         | Robot movement                                    |
| Voltage Regulator    | Stabilizes voltage for sensitive components       |
| UART Communication   | Link between Raspberry Pi and ESP32               |
| 2WD / 4WD Chassis    | Physical structure of the robot                   |
| Servo motor          | Controls steering direction                       | 


Software & Libraries Overview
|    Platform         |   Language / Frameworks          |   Key Functions / Roles                          |
|---------------------|----------------------------------|--------------------------------------------------|
| Raspberry Pi 4      | Python 3                         | Main control logic, image processing             |
|                     | OpenCV                           | Computer vision (line following, object tracking)|
|                     | pyserial                         | UART communication with ESP32                    |
|                     | numpy, imutils                   | Math operations, image transformations           |
| ESP32               | MicroPython                      | Lightweight firmware for real-time control       |
|                     | machine, time, ustruct           | Sensor reading, PWM generation, UART handling    |
|                     | PID function                     | Servo control                                    |


Workflow Table
|    Module           |    Function / Task                                      |
|---------------------|---------------------------------------------------------|
| Raspberry Pi 4      | Captures image via camera                               |
|                     | Processes image using OpenCV (color)                    |
|                     | Determines direction.                                   |
|                     | Sends commands to ESP32 via UART                        |
| ESP32               | Receives commands from Raspberry Pi                     |
|                     | Reads ultrasonic sensor data                            |
|                     | Applies PID control to maintain safe distance           |
|                     | Controls motors and servo using PWM                     |



[Menu](#Contents)

[Menu](#Contents)
___
