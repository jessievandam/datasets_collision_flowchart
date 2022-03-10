## Datasets description
All datasets are located in the folder `/datasets`. The information related to each dataset is displayed in Table 1 below. Each dataset consists of one .mat file and one .txt file. The .txt file contains a description of the contact location and direction of the applied collisions. The .mat file stores the following variables, that are required as an input for the collision-event pipeline (note that the size is the size of the vector/matrix/double at one time instant):

| **Description**                                                                                                          	| **Unit** 	| **Size** 	| **Matlab name**          	|
|--------------------------------------------------------------------------------------------------------------------------	|----------	|----------	|--------------------------	|
| Generalized coordinates                                                                                                  	| rad      	| 24x1     	| `q`                      	|
| Generalized velocities                                                                                                   	| rad/s    	| 24x1     	| `qd`                     	|
| Desired joint velocities                                                                                                 	| rad/s    	| 18x1     	| `qd_des`                 	|
| Measured motor torques                                                                                                   	| Nm       	| 18x1     	| `taum`                   	|
| Inertia matrix                                                                                                           	| kg m^2   	| 24x24    	| `massMatrix`             	|
| Nonlinear terms                                                                                                          	| Nm       	| 24x1     	| `nonlinearTerms`         	|
| Magnitude ground truth collision force                                                                                   	| N        	| 1x1      	| `magFTforce`             	|
| Stacked translational feet Jacobians                                                                                     	| -        	| 12x24    	| `jacobiansFeet`          	|
| Spatial colliding link Jacobians<br>(order of Jacobians in cell: upperarm, <br>forearm, wrist 1, wrist 2, gripper, base) 	| -        	| 6x24     	| `jacobiansCollidingLink` 	|
| Time                                                                                                  	| sec      	| 1x1     	| `time`                      	|
| Force measured by force/torque<br>(F/T) sensor at the end-effector                                                                                                  	| N      	| 3x1     	| `forceEE`                      	|

A few additional notes on the datasets:
- The number of collisions in the table sum up to 425, while in the paper we refer to 416 experiments. No ground truth F/T sensor collision force is available for datasets 6 and 14, thus these datasets are not analyzed in the paper. Nevertheless, this data can still be used to test collision detection methods.  
- The number of collisions for dataset 13 is 22 in the table, but 23 in the .txt file. One of the collisions is incorrectly recorded in the ground truth F/T sensor data. This collision is shown in the estimated force, but not in the ground truth force. 

![Table](table.jpg)


## Code description
The provided scripts have been tested in MATLAB R2020b.
The parameters used in the code are the following (note that the size is the size of the vector/matrix/double at one time instant):
| **Description**                                                                                                                                                   	| **Unit** 	| **Size** 	| **Matlab name**        	|
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------	|----------	|----------	|------------------------	|
| End index of time vector                                                                                                                                          	| -        	| 1x1      	| `endInd`               	|
| Sampling time                                                                                                                                                     	| sec      	| 1x1      	| `Ts`                   	|
| Linearly spaced time vector                                                                                                                                       	| sec      	| 1x1      	| `timeVec`              	|
| Estimated external torques                                                                                                                                        	| Nm       	| 24x1     	| `torques`              	|
| Estimated external force                                                                                                                                          	| N        	| 3x1      	| `force`                	|
| Estimated external force with added <br>low-pass filter (LPF)                                                                                                      	| N        	| 3x1      	| `forceLPF`             	|
| Cut-off frequency LPF                                                                                                                                             	| Hz       	| 1x1      	| `fc`                   	|
| If the second peak doesn't appear after <br>$T_\text{2peaks}$ sec, the ending of the collision <br>is detected                                                    	| sec      	| 1x1      	| `T_twopeaks`           	|
| If all force components are below the <br>threshold for $T_{\text{rippling}}$ sec after the <br> collision has ended, the collision has <br>disappeared 	        | sec      	| 1x1      	| `T_rippling`           	|
| Minimum cut-off frequency band-pass<br>filter (BPF)                                                                                                               	| Hz       	| 1x1      	| `cutOffFreqMin`        	|
| Maximum cut-off frequency BPF                                                                                                                                     	| Hz       	| 1x1      	| `cutOffFreqMax`        	|
| Constant threshold for $x, y$ and $z$<br>components of filtered force                                                                                             	| N        	| 3x1      	| `constThresh`          	|
| Collision bool                                                                                                                                                    	| -        	| 1x1      	| `collision`            	|
| Estimated collision force including <br>disturbances                                                                                                              	| N        	| 3x1      	| `forceEstimated`       	|
| Estimated collision force excluding<br>disturbances                                                                                                               	| N        	| 3x1      	| `forceCollision`       	|
| Magnitude estimated collision force<br>excluding disturbances                                                                                                     	| N        	| 3x1      	| `magEstForceCollision` 	|
| Estimated disturbance force                                                                                                                                       	| N        	| 3x1      	| `disturbance`          	|

The steps of the collision pipeline, followed in the sample code found in `main.m` are the following (please refer to the paper "Collision detection, isolation and identification for a legged manipulator" for more detailed explanations):
1. **External torque estimation.** In the file `momentum_observer.m`, the continuous-time momentum-based observer for floating-base robots is implemented, resulting in the estimated torques.
2. **External force estimation.** In the file `estimate_force_base_arm.m`, the stacked force vector containing the linear forces at the four feet of the legged manipulator, and the force on the colliding body part, are computed. The output is `force` (or `forceLPF` for estimation during trotting), which is a 2x1 cell containing the estimated force on the arm and base. 
3. **Collision detection.** In `collision_detection.m`, collisions are detected. The bool `collision` indicates if a collision occurs for each time instant, with either a 0 or a 1. 
4. **Collision identification.** In `collision_identification.m` the disturbance forces are computed and subtracted from the estimated forces, resulting in the final estimated collision force. The magnitude of this final collision force is calculated such that it can be plotted against the ground truth F/T collision force. 

To extract and plot the collision data for a specific dataset, one needs to run the script `main.m`. A dataset can be selected by setting the variable `datasetNr` equal to the corresponding number (e.g., `datasetNr=1`) at the beginning of the script.
