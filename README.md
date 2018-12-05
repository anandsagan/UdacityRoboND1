[//]: # (Image References)
[image_0]: ./misc/rover_image.jpg
[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Search and Sample Return Project


![alt text][image_0] 

This project is modeled after the [NASA sample return challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html) and it will give you first hand experience with the three essential elements of robotics, which are perception, decision making and actuation.  You will carry out this project in a simulator environment built with the Unity game engine.  

## The Simulator
The first step is to download the simulator build that's appropriate for your operating system.  Here are the links for [Linux](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Linux_Roversim.zip), [Mac](	https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Mac_Roversim.zip), or [Windows](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Windows_Roversim.zip).  

You can test out the simulator by opening it up and choosing "Training Mode".  Use the mouse or keyboard to navigate around the environment and see how it looks.

## Dependencies
You'll need Python 3 and Jupyter Notebooks installed to do this project.  The best way to get setup with these if you are not already is to use Anaconda following along with the [RoboND-Python-Starterkit](https://github.com/ryan-keenan/RoboND-Python-Starterkit). 


Here is a great link for learning more about [Anaconda and Jupyter Notebooks](https://classroom.udacity.com/courses/ud1111)

## Recording Data
I've saved some test data for you in the folder called `test_dataset`.  In that folder you'll find a csv file with the output data for steering, throttle position etc. and the pathnames to the images recorded in each run.  I've also saved a few images in the folder called `calibration_images` to do some of the initial calibration steps with.  

The first step of this project is to record data on your own.  To do this, you should first create a new folder to store the image data in.  Then launch the simulator and choose "Training Mode" then hit "r".  Navigate to the directory you want to store data in, select it, and then drive around collecting data.  Hit "r" again to stop data collection.

## Data Analysis
Included in the IPython notebook called `Rover_Project_Test_Notebook.ipynb` are the functions from the lesson for performing the various steps of this project.  The notebook should function as is without need for modification at this point.  To see what's in the notebook and execute the code there, start the jupyter notebook server at the command line like this:

```sh
jupyter notebook
```

This command will bring up a browser window in the current directory where you can navigate to wherever `Rover_Project_Test_Notebook.ipynb` is and select it.  Run the cells in the notebook from top to bottom to see the various data analysis steps.  

The last two cells in the notebook are for running the analysis on a folder of test images to create a map of the simulator environment and write the output to a video.  These cells should run as-is and save a video called `test_mapping.mp4` to the `output` folder.  This should give you an idea of how to go about modifying the `process_image()` function to perform mapping on your data.  

## Navigating Autonomously
The file called `drive_rover.py` is what you will use to navigate the environment in autonomous mode.  This script calls functions from within `perception.py` and `decision.py`.  The functions defined in the IPython notebook are all included in`perception.py` and it's your job to fill in the function called `perception_step()` with the appropriate processing steps and update the rover map. `decision.py` includes another function called `decision_step()`, which includes an example of a conditional statement you could use to navigate autonomously.  Here you should implement other conditionals to make driving decisions based on the rover's state and the results of the `perception_step()` analysis.

`drive_rover.py` should work as is if you have all the required Python packages installed. Call it at the command line like this: 

```sh
python drive_rover.py
```  

Then launch the simulator and choose "Autonomous Mode".  The rover should drive itself now!  It doesn't drive that well yet, but it's your job to make it better!  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results!  Make a note of your simulator settings in your writeup when you submit the project.**

## My Contributions
There were two parts to this project that were to be fulfilled, the Notebook analysis and the autonomous navigation and mapping. Majority of the analysis was first testing in the `process_image()` in the Rover notebook and transferred into the perception_step function. The decision_step function provides the rover with the best course of action given the circumstance.

### Notebook Analysis
The obstacle identification that I implemented is fairly straighforward.  The `color_thresh` function of `perception.py` returns the path as well as the obstacle. Here is the part of the code that implements obstacle detection: ```sh
    below_thresh = (img[:,:,0] < rgb_thresh[0]) \
                 & (img[:,:,1] < rgb_thresh[1]) \
                 & (img[:,:,2] < rgb_thresh[2])
```
It uses the same threshold as the path detection and wherever it was below the specified RGB threshold (160,160,160), it's an obstacle. The `find_rock` function in the `perception.py` file contains the method to find a rock. I implemented an HSV filter to identify the rocks in the warped image. The lower and upper limits of the yellow filter in HSV are [90,100,100] and [100,255,255], respectively. Here's the code for rock detection: ```sh
def find_rock(img, lower_thresh, upper_thresh):
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                  [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,lower_thresh,upper_thresh)
    res = cv2.bitwise_and(img,img, mask = mask)
    return mask, res
```
Then using the coordinates for the rocks, path, and obstacles, the `pix_to_world` function returns a worldmap of the all the components. The world map shows the path in red, obstacles in blue, and rocks in white. The output video can be found in the output folder of this repository.

### Autonomous Navigation and Mapping
The `perception_step()` is similar to the process_image() function in the Rover test notebook, but it only contains the key variables and elements. It returns Rover, which updates the values in `drive_rover.py` so it can be accessed by `decision.py`. 

The `decision_step()` function determines what the rover does when Rover is updated. The key functionalities I added allows the rover to navigate to and pick up rocks when they are visible. If there are no rocks present and there is room to go forward, the throttle is set to 0.3 and steers towards the average direction of the visible path. If there's no room to go forward, the rover stops and turns 15 degrees until it can seem some of the path.

There are two main modes for the Rover, forward and stop. It goes forward when there is room to move and stops when there isn't. Sometimes, when the rock is in a corner, the Rover.mode will be stop, but it will keep incrementing forward until it has picked up the rock.









