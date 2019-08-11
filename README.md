# Camera Calibration

Implementation of calibrating intrinsic and extrinsic camera parameter
with pso optimization and quaternion rotation for distance calculation .


## Dependencies

```bash
sudo apt install libopencv-dev cmake
```

## Environment Setup

First, set the camera in a fixed position then place the chessboard 
in front of it at a fixed distance, as shown below :

<p align="center">
<img src = "https://github.com/ErfanFathi/camera-calibration/blob/master/utils/setup.jpg"</img>
</p>

## Compilation 

```bash
cmake .
make all
./Calibration
```

## Code Setup

###### In "main.cpp" file i write some comments which that helps you ... <br />

#### 1) Input Real Points :

generally you should first set the actual distance in "realPoint" array in main.cpp file
just note that the array is as follows :

<p align="center">
<img src = "https://github.com/ErfanFathi/camera-calibration/blob/master/utils/configuration.jpg"</img>
</p>

#### 2) Set Optimization Parameter :

This part of process very important because pso algorithm 
highly parameter based so pay attention to the parameters. <br />

## How To Use ?

After the setup environment and code, compile and run code .
if print "pattern not found" there is a problem in your chessboard
check that if print "pattern found" that's ok . after calibration process you have 10 parameter .
now you can use that and calculate distance or pixel .
for this work there is two function : <br />
#### F1) Field2Image
Field2Image function get distance "milimeter" and calculate pixel .
#### F2) Image2Field
vice versa :)

## Detected Corners

<p align="center">
<img src = "https://github.com/ErfanFathi/camera-calibration/blob/master/utils/actual_corners.jpg"</img>
</p>

## Projected Corners After Calibration

<p align="center">
<img src = "https://github.com/ErfanFathi/camera-calibration/blob/master/utils/projected_corners.jpg"</img>
</p>

## Euler angles and quaternions

If you want read more about euler angles and quaternions i suggest you read the following paper : <br />
[A Tutorial on Euler Angles and Quaternions](http://www.weizmann.ac.il/sci-tea/benari/sites/sci-tea.benari/files/uploads/softwareAndLearningMaterials/quaternion-tutorial-2-0-1.pdf)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

## Finale

This is a message for github : <br /> 
Yeah i'm a iranian scientist <br />
but you can't restricted my freedom <br />
you can't restricted my github account <br />

Feel free to use, modify this code. And please feel free to fork the code 
from Github and send pull requests.

Report any comment or bugs or question to:<br />
fathierfan97@gmail.com<br />

Regards,<br />
Erfan Fathi<br />
