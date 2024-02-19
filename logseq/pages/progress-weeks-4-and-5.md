- Progress from 2024-02-05 to 2024-02-16
- # 2024-02-05
  
  * Investigating how to import meshes into simulation to make it more realistic
  * Converted action space to spherical coordinates rather than cartesian to allow for better actions.
- # 2024-02-07
  
  * Held round-table discussion with advisors
  * Downloading **meshlab** to begin calculating inertial parameters.
- # 2024-02-09
  
  $$I_{\text{actual}} = I \cdot \frac{1}{s^2} \frac{m_{actual}}{V_{\text{from meshlab}}}$$
  
  Where $s$ is the scale, $m_{\text{actual}}$ is the measured mass in real life, and $V_{\text{from meshlab}}$ is the volume reported by meshlab.
  
  * Learned how to validate robot description files in the (Universal Robotic Description Format)
  ```sh
  check_urdf <(xacro model.urdf.xacro)
  ```
  * Added all 10 rangefinders into simulation but their orientations are incorrect.
- # 2024-02-11
  
  * I got the rangefinders simulated and hacked them to view outside of the bounding box of the drone.
  ![scrot](https://github.com/ReadyResearchers-2023-24/syllabus/assets/56975504/e2c05e45-6256-4979-9090-b1940b3867ac)
  * `pitch="${pi / 2}"` corresponds to facing downwards.
  * Disocvered that Materials can be defined with scripts.
- # 2024-02-12
  
  * Troubleshooting issues with the pose estimator (Extended Kalman Filter)
  * Forwarding rangefinder information to FCU to calibrate when it has touched the ground.
- # 2024-02-13
  
  * Discovery that FCU uses camera for position estimation.
  * Taking steps backward..
- # 2024-02-14
  
  * Beginning to look into how to re-implement position estimation using VIO. As one user said:
  
  > [ppoirier](https://discuss.ardupilot.org/u/ppoirier)
  > 
  > [Jun '23](https://discuss.ardupilot.org/t/indoor-position-estimate-with-onboard-sensors-only/102526/3)
  > Centimetric gps denied localization is the Holy Grail of UAV.
  > Get ready to spend time and money if you intend to achieve this goal at the centimetric level.
  > 
  > You can look here for some inspiration
  > [https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html 23]> 
  > (https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html)
  
  * Moving pose estimation camera to account for sensor array drum.
- # 2024-02-15
  
  * Meeting with JJ and discussing which experimental metrics to display after running experiments.
  * Discussion with Dr. Lombardi for using server computers (CUDA enabled) for my simulation and training