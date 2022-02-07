# PT-Camera-model

Designed mathematical model of pan-tilt camera to simulate its bounded field-of view. this model can simulate its field-of-view (FOV) on the camera position and orientation, as well as the target position in the region of interests (ROI). To keep the target in the center area of the camera’s FOV, a simple PID controller was designed to control the desired pan and tilt angle. After the movement of the target over time, the target is still in the center area of the camera’s FOV.

### Result

Here is the result, see [simulation video](https://www.youtube.com/watch?v=DLapzyhzNdk): 

<img width="547" alt="Screen Shot 2022-02-06 at 7 46 30 PM" src="https://user-images.githubusercontent.com/94488336/152709108-79db2cbc-fc91-4da5-8ce0-7b78f636e464.png">
