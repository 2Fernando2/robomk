### Actividad 2 ...
IGUAL
The primary objective of this activity is to transition the robot control system from a purely reactive navigation model (Activity 1) to an **autonomous self-localization system**. This requires the robot to continuously estimate its full pose, including both translation and rotation, withing a known rectangular room defined by its four corners.

IGUAL
The transition necessitated a significant architectural change: the reactive Chocachoca state machine was replace by the new **Localiser** component. The core task of the Localiser is to estimate the robot's pose (r, phi) by minimizing an error function that considers the differences between the nown (nominal) corner positions (c_i) and the corner positions estimated from each sensor measurement (c_i^m). This process provides the basis for sophisticated path planning.

IGUAL
The localiser focuses on integrating external libraries like **OpenCV** and the **Hungarian Algorithm** to process LiDAR data against an internal map representation.

#### Self-localisation methodology
CAMBIOS
The localisation process is computed within the new Localiser component and follows a three stage pipeline: Feature Extraction, Feature Matching and Pose Estimation.

##### Feature Extraction
CAMBIOS
The system relies on a **Line-Based Localizaiton** strategy. Raw point cloud data from the Lidar3D component is processed to extract geometric features of the environment. A **Room Detector** component is used to perform a line extraction on the raw LiDAR data using the **RANSAC** algorithm.

\begin{equation}
ax + by + c = 0 \quad \text{where } a^2 + b^2 = 1
\label{eq:ransac_line_model}
\end{equation}

The extracted lines are checked for a approximate 90º intersections, defining the set of **measured corner points** used for matching.

##### Feature Matching
CAMBIOS
Before calculating the new pose, the measured features must be reliably associated with the known features of the environment model. This is achieved using the **Hungarian Algorithm** to find the best possible association between the measured corner points and nominal corner points. For each pair, the Euclidean distance is computed and added to the cost matrix. The cost matrix is defined as:

\begin{equation}
C = \begin{bmatrix}
d_{11} & d_{12} & \cdots & d_{1m} \\
d_{21} & d_{22} & \cdots & d_{2m} \\
\vdots & \vdots & \ddots & \vdots \\
d_{n1} & d_{n2} & \cdots & d_{nm}
\end{bmatrix}
where $d_{ij}$ is the Euclidean distance between the $i$-th measured corner point and the $j$-th nominal corner point.
\end{equation}

This algorithm minimizes the total cost of correspondence, ensuring optimal paring between the two sets of points. To create the aligned frame, all corners are first transformed into a single common reference frame (the room's coordinate system) using the current estimated pose.

##### Pose Estimation
CAMBIOS
The final stage computes the best-fit pose transformation that aligns the matched corners sets. This is achieved firstly linearizing the pose estimation problem into a system of linear equations. 

\begin{equation}
\mathbf{c}_i \approx \mathbf{R}_phi \, \mathbf{m}_i + \mathbf{t}_r
\label{eq:measurement_model}
\end{equation}

where $\mathbf{c}_i$ is the $i$-th measured corner point, $\mathbf{R}_phi$ is the rotation matrix, $\mathbf{m}_i$ is the $i$-th nominal corner point and $\mathbf{t}_r$ is the translation vector.

The unknown vector contains the three degrees of freedom of the robot's pose. The linear equation is solved using the **Pseudoinverse** method to obtain the 3D vector, which represents the new pose estimation. This yields the linearized matrix:

\begin{equation}
\mathbf{R}_\phi \approx
\begin{bmatrix}
1 & -\phi \\
\phi & 1
\end{bmatrix}
\label{eq:linearized_rotation}
\end{equation}

For each correspondence, we can write the linear relationship as:

\begin{equation}
\begin{bmatrix}
1 & 0 & -m_{i,y} \\
0 & 1 & m_{i,x}
\end{bmatrix}
\begin{bmatrix}
x \\
y \\
\phi
\end{bmatrix}
=
\begin{bmatrix}
c_{i,y} - m_{i,y} \\
c_{i,x} - m_{i,x}
\end{bmatrix}
\label{eq:linearized_measurement}
\end{equation}

The pose estimation problem is linearized and formulated as a system of equations:

\begin{equation}
\mathbf{W} \cdot \mathbf{r} = \mathbf{b}
\label{eq:linear_system}
\end{equation}