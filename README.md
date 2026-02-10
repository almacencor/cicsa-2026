# Introduction
We are the StormsNGR team from Hungary, and we are competing in the 2025 WRO Future Engineers category. This is our second year competing after a very successful first year, for which you can find our documentation [here](https://github.com/MoCsabi/WRO2024-FE-StormsNGR). We will be keeping that repository as it is, and will be updating this repository for the 2025 season.

## The team
>Team members
- **Csaba Molnár** from [Budapest University of Technology and Economics](https://www.bme.hu/en) Faculty of Electrical Engineering and Informatics
  - csabi@molnarnet.hu
- **András Gräff** from [Budapest University of Technology and Economics](https://www.bme.hu/en) Faculty of Mechanical Engineering
  - andrasgraff@gmail.com
- **Levente Molnár** from [Sóskút Károly Andreetti elementary school](https://iskola.soskut.hu/)
  - levente@molnarnet.hu
>Coach
- József Balázs Gräff
  - graffjozsefb@gmail.com

Links to our socials:

- **Facebook**: https://www.facebook.com/stormsteam/
  - Here you can find updates about the team
- **YouTube**: https://www.youtube.com/channel/UCyzm_Su7qoRCof-ZpbG_9Ig
  - Here you can watch videos about past competitions
- **Instagram**: https://www.instagram.com/storms_team_hun/
  - Cool posts and updates about our, and our sister-team StormsRMS' preparation for upcoming competitions
<img width="842" height="468" alt="image" src="https://github.com/user-attachments/assets/f008d91d-e2a0-4c5f-97a4-c56cfcc7fe91" />

This is the official repository of the Team CICSA for the international final of the WRO2025 season in Panamá.


## Follow us!

| Facebook | YouTube | Instagram |
|---------------|---------------|----------------|
| [![Facebook](other/facebook.jpg)](https://www.facebook.com/share/1A1hz6zQSn/) | [![YouTube](other/youtube.png)](https://www.youtube.com/@CICSA_Academia) | [![Instagram](other/instagram.jpg)](https://share.google/ZzGMFZOurGsjhWN2D) |


___
## The team.
====

CICSA WRO 2025 - Future Engineers

Coach **Sergio Iván Hernández Ruiz**

Coach Sergio Iván provides the technical guidance and leadership required to keep Team CICSA on track. With extensive experience in robotics, he helps us navigate complex challenges, refine our designs, and develop solutions that work in competitive and real-world settings.

Sergio Iván has been the director of the CICSA robotics academy since 2015 and has participated in events organized by the WRO since 2019.
***
**Members:**

**Gildardo Garcia**, 
***
**Sergio Amid Hernández Gónzalez**, at 19 years old, he studies Software Engineering at Kuepa University. He has participated in numerous competitions, including: the 2019 WRO Regional in Guadalajara, Jalisco, México, where he obtained 1st place. In 2024, in the city of Mexicali, Baja California, México, they obtained 1st place in the preparatory category. He was part of the Mexican robotics team that represented Mexico at the ChampionShip in Italy in 2024.
***
**Diego Pereida Arochi**,

___

# Abstract
Our solution is an autonomous car powered by a Raspberry Pi coded in Python, responsible for the main challenge logic, and an ESP microcontroller coded in C, responsible for controlling the motors and processing sensor data. The two devices communicate using UART. We use a 360° 2D LiDAR combined with a gyro to always know where the robot is on the mat. Detection of the traffic sign's color is done by combining the output of the LiDAR with the camera feed to know exactly where the object is. We use ackermann steering geometry calibrated so the car can leave the parking space in one continuous arc. Driving is powered by 1 DC motor through a differential gearbox designed from scratch, with a top speed of around 2.6 m/s.
@@ -36,6 +46,3 @@ Our solution is an autonomous car powered by a Raspberry Pi coded in Python, res
### [Videos](/video/video.md)
### [Team Photos](/t-photos/)
### [Vehicle Photos](/v-photos/)

# Special thanks
Special thanks to Zsolt Molnár for all the mentoring he's done for us, József Molnár from the [Budapest University of Technology and Economics](https://www.bme.hu/) Faculty of Mechanical Engineering for helping us with the preparations, György Fenyvesi for helping us develop our prototype [custom made interconnect panel](/schemes/README.md/#custom-made-interconnect-panel-wiring-with-connections-labeled), and a bunch of others for helping us by reviewing all the documentation. And of course, to both our families for tolerating us taking up the entire living room with the robot mat 🙂.
