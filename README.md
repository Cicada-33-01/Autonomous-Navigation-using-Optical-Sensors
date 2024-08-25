
# Autonomous Navigation using optical mouse sensor

We have endeavoured to make a bot which is capable of dead reckoning in indoor
settings. In this context, dead reckoning is the task of accurately estimating the position of the bot along the x and y axis with respect to the initial orientation (origin) using optical flow sensors (OFS) extracted from common gaming mouse. 

Uniqueness of our solution lies in providing highly accurate orientation using low cost OFS sensors with assisstance of Inertial Measurement Unit (IMU) and wheel encoder without the help of any external communication. Conventional options includes GPS and GNSS but are inaccurate for small distance traversing (< 5m) or  LiDAR and Time of Flight sensors (TOF) which are costly. 

The reason of using IMU and wheel encoder is that low cost OFS sensors create data with high error when not perfectly aligned with ground and relying solely on one type of sensor would simply be not enough for accurate results.

## Computing distance traversed using OFS

The two optical flow sensors (OFS) are indicated by points \( L \) and \( R \) in the schematic shown in Figure 1\. The point \( O \), which is the mid-point of \( L \) and \( R \), is designated as the center of the robot. Let the readings from the two sensors at \( L \) and \( R \) in the current step be (Δx_L, Δy_L)  and (Δx_R, Δy_R) . These are indicated in Figure by arrows shown in blue.

![image](https://github.com/user-attachments/assets/349a99a6-9983-45b1-a219-1bbea3919723)


Let the local coordinate system (LCS) of the robot be centered at the point \( O \). The displacement (Δx, Δy)  of the point \( O \) in the LCS is simply the average of individual components.

In order to compute \( Δθ), we note that the new positions of \( L \) and \( R \) in the LCS are \( L' = L + (Δx<sub>L</sub>, Δy_<sub>L</sub>)  and \( R' = R + (Δx<sub>R</sub>,  Δy<sub>R</sub>) \). Then \( Δθ ) may be obtained as

Δθ= atan2(Δy_R - Δy_L , Δx_R - Δx_L).



( Δx<sub>i</sub>,  Δy<sub>i</sub>, Δθ=<sub>i</sub>)


Sensors kaun kaunse use kiye and how they function
PCB 

How to run code, in order to ensure robot to run, power up arduino and load the file getting data and RPI with main file 


