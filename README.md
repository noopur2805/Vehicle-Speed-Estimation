# Vehicle Speed Estimation

To execute the code:
```
`python speed_estimator.py -i <input_filename> -o <output_filename>
```

Code Files:

1. vanishing_points_3.py

This file has been used to get the vanishing points.

2. speed_estimator.py

Steps:
* Obtained calibration lines only out of the image. Here I used road demarcations as calibration (demarcations are by standard 3m in length with 4.5m gap).
* Obtained lane contours. (Used frame0.jpg – a blank image with no vehicle for background substraction)
* Used these demarcations along with the vanishing point and lane contour to adjust perspective.
* Then found the intersection points of the [lane contours from its vanishing point on right] and [road demarcations encapsulated within the vanishing points on its left].
* This gave a quadrilateral which was then extended to fit onto original image vertices. (Perspective changing step). Due to absence of camera calibration parameters, certain level of distortion still persisted.
* Now, with line connecting demarcations and vanishing point becoming parallel to X-axis here, calculated speed for each detected moving vehicle by the formula:
    * time = total time for which vehicle was within ROI * frames_per_sec
    * distance = pixel distance along Y-axis in ROI * actual length
    * Speed =  distance / time
