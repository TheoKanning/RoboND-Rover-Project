## Project: Search and Sample Return
---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg
[warped]: ./writeup_images/warped.jpg 
[obstacle]: ./writeup_images/obstacle.jpg 
[navigable]: ./writeup_images/navigable.jpg 

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Obstacle Detection
An RGB threshold was applied to each image after the perspective transform in order to find obstacles. Since the obstacles 
were all lighter than the drivable surface, any pixel with RGB values all over 160 was assigned as a drivable area. Any
pixel that was not drivable was considered an obstacle. Any values behind the rover's field of view was considered to be
 neither an obstacle nor a navigable area. For example, look at the following image after a perspective transform. 
 
 *Warped Image*
 
 ![Warped][warped]
 
 
 Notice the black areas in the bottom left and right. These pixels can't be seen by the camera, so they appear black after 
 applying the perspective transform.
 
 *Navigable Area*
 
 ![Navigable]
 
 This is the navigable area. These pixels are identified by having RGB values all above 160.
 
 *Obstacles*
 
 ![Obstacle]
 
 The obstacle area is the inverse of the navigable area, except for the bottom left and right areas. After taking the 
 inverse of the navigable area, any pixel that is (0,0,0) in the warped image is set to 0 here.
 
 

#### 2. Sample Detection
Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 
And another! 

![alt text][image2]
### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  

Improvements:
Better RGB threshold to include shadowy areas because my algorithm does a good job of avoiding walls anyway

![alt text][image3]


