# Estimation Project #

In this estimation project, we develop the estimation portion of the controller used in the CPP simulator.  By the end of the project, the simulated quad will be flying with an estimator and a custom controller (from the controls project)

This README is broken down into the following sections:

 - [Setup](#setup) - the environment and code setup required to get started and a brief overview of the project structure
 - [The Solutions to the tasks](#the-tasks-solutions) - the tasks you will need to complete for the project
 - [References](#references) - The reference documents used for completing the project.

## Setup ##

This project will continue to use the C++ development environment set up in the Controls C++ project.

 1. Clone the repository
 ```
 git clone https://github.com/udacity/FCND-Estimation-CPP.git
 ```

 2. Import the code into your IDE like done in the [Controls C++ project](https://github.com/udacity/FCND-Controls-CPP#development-environment-setup)
 
 3. Compile and run the estimation simulator just as in the controls project

## Note

All the videos and results of scenarios 1 to 6 are taken with the custom controller from the controls project.
The good news is that all scenarios from 6 to 11 pass using my [custom controller](https://github.com/anupamberi/FCND-Controls-CPP/blob/master/src/QuadControl.cpp)
## The Tasks Solutions ##

Here is the list of tasks and the to do lists. Each of these tasks is followed by a solution after the implementation/tuning.
Project outline:

 - [Step 1: Sensor Noise Solution](#step-1-sensor-noise-solution)
 - [Step 2: Attitude Estimation Solution](#step-2-attitude-estimation-solution)
 - [Step 3: Prediction Step Solution](#step-3-prediction-step)
 - [Step 4: Magnetometer Update Solution](#step-4-magnetometer-update)
 - [Step 5 & 6 : Closed Loop + GPS Update Solution + Custom Controller](#step-5--6-closed-loop--gps-update-solution--custom-controller)

### Step 1: Sensor Noise Solution

The sensor measurements are stored in the files `config/log/Graph1.txt` and `config/log/Graph2.txt`. The scenario file is loaded in the `main` function `LoadScenario(_scenarioFile)`.
We have to ensure that the standard deviations are calculated well before the loading scenario step. 

Declare the following constants,

```
// Scenario 1 constants
const string SENSORNOISE_PATH = "../config/06_SensorNoise.txt";
const string GRAPH1_PATH = "../config/log/Graph1.txt";
const string GRAPH2_PATH = "../config/log/Graph2.txt";
```
And check if the previous scenario is `06_SensorNoise.txt`. Compute standard deviations and print in stdout.
The modified [main](./src/main.cpp#L53-L88) function looks like,

```
int main(int argcp, char **argv)
{
  PrintHelpText();
 
  // load parameters
  ParamsHandle config = SimpleConfig::GetInstance();

  // initialize visualizer
  visualizer.reset(new Visualizer_GLUT(&argcp, argv));
  grapher.reset(new GraphManager(false));

  // re-load last opened scenario
  FILE *f = fopen("../config/LastScenario.txt", "r");
  if (f)
  {
    char buf[100]; buf[99] = 0;
    fgets(buf, 99, f);
    _scenarioFile = SLR::Trim(buf);
    fclose(f);
  }

  if (_scenarioFile.compare(SENSORNOISE_PATH) == 0) {
    // Read Graph1.txt for GPS X data and set standard deviation
    printStandardDeviation(GRAPH1_PATH);
    // Read Graph2.txt for Accel X data and set standard deviation
    printStandardDeviation(GRAPH2_PATH);
  }

  LoadScenario(_scenarioFile);
 
  glutTimerFunc(1,&OnTimer,0);
  
  glutMainLoop();

  return 0;
}
```
And the function printStandardDeviation is defined as [printStandardDeviation](./src/main.cpp:L98-L122).

Upon the first run, the standard deviations for the GPS X and Accelerometers X position is calculated and printed 
the stdout. Set the respective values in `MeasuredStdDev_GPSPosXY` and `MeasuredStdDev_AccelXY` variables. 
Upon reopening the simulator, the scenario passes with 68% measurements getting within the computed standard deviation.

![Scenario 1](./images/scenario_6.gif)

### Step 2: Attitude Estimation Solution ###

The solution is based on the section 7.1.2 of the [Estimation for Quadcopters](https://www.overleaf.com/read/vymfngphcccj).

We use the `Quaternion<float>` class to create a rotation quaternion from the current Euler angles and then integrate the 
Gyroscope body rates using `IntegrateBodyRate(gyro, dtIMU)`.

The function [UpdateFromIMU](./src/QuadEstimatorEKF.cpp#L74-L120) is modified as below,

```
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  Quaternion<float> qt = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
  // Creat a Quaternion dq from the measurement of the angular rates from IMU in the body frame
  Quaternion<float> qtBar = qt.IntegrateBodyRate(gyro, dtIMU);

  float predictedPitch = qtBar.Pitch();
  float predictedRoll = qtBar.Roll();
  ekfState(6) = qtBar.Yaw();

  // normalize yaw to -pi .. pi
  if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
  if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;

  /////////////////////////////// END STUDENT CODE ////////////////////////////
```
![Scenario 2](./images/scenario_7.gif)

### Step 3: Prediction Step Solution ###

The solution of the Predict step is based on section 7.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj).

We make use of the Rbg rotation matrix and compute the predicted step as

`predictedState = a + b * ut`, where a and b are replaced by the relevant Transition model variants.

The  [PredictState](./src/QuadEstimatorEKF.cpp#L145-L217) is updated as below,

```
////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  VectorXf a(7);

  a(0) = predictedState(0) + predictedState(3) * dt;
  a(1) = predictedState(1) + predictedState(4) * dt;
  a(2) = predictedState(2) + predictedState(5) * dt;
  a(3) = predictedState(3);
  a(4) = predictedState(4);
  a(5) = predictedState(5) - CONST_GRAVITY * dt,
  a(6) = predictedState(6);

  MatrixXf b(7, 4);
  b.setZero();

  float phi = rollEst;
  float theta = pitchEst;
  float psi = curState(6);

  // Assign first row of Rbg
  b(3, 0) = cos(theta) * cos(psi);
  b(3, 1) = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
  b(3, 2) = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
  // Assign second row of Rbg
  b(4, 0) = cos(theta) * sin(psi);
  b(4, 1) = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
  b(4, 2) = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
  // Assign third row of Rbg
  b(5, 0) = - sin(theta);
  b(5, 1) = cos(theta) * sin(phi);
  b(5, 2) = cos(theta) * cos(phi);

  b(6, 3) = 1.0;

  // Define the control input vector
  VectorXf ut(4);
  ut.setZero();
  ut(0) = accel.x;
  ut(1) = accel.y;
  ut(2) = accel.z;
  ut(3) = 0.0;

  ut = ut * dt;

  // Compute predicted state
  predictedState = a + b * ut;

  /////////////////////////////// END STUDENT CODE ////////////////////////////
``` 

When we run scenario `08_PredictState` the estimator state tracks the actual state, with only reasonably slow drift, as shown in the figure below:

![Scenario 3](./images/scenario_8.gif)

The [GetRbgPrime](./src/QuadEstimatorEKF.cpp#L219-L254) is similarly updated as,

```
 ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // Define the control vector in terms of the accelerations in X,Y & Z directions
  VectorXf ut(3);
  ut.setZero();

  ut(0) = accel.x;
  ut(1) = accel.y;
  ut(2) = accel.z;

  // Set the dt values in gPrime
  gPrime(0, 3) = dt;
  gPrime(1, 4) = dt;
  gPrime(2, 5) = dt;

  ut = ut * dt;

  gPrime(3, 6) = RbgPrime.row(0) * ut;
  gPrime(4, 6) = RbgPrime.row(1) * ut;
  gPrime(5, 6) = RbgPrime.row(2) * ut;

  ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;

  /////////////////////////////// END STUDENT CODE ////////////////////////////
```
Upon the implementation of `PredictState` and `GetRbgPrime`, the [Predict](./src/QuadEstimatorEKF.cpp#L256-L319) function is updated as,

```
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // Define the control vector in terms of the accelerations in X,Y & Z directions
  VectorXf ut(3);
  ut.setZero();

  ut(0) = accel.x;
  ut(1) = accel.y;
  ut(2) = accel.z;

  // Set the dt values in gPrime
  gPrime(0, 3) = dt;
  gPrime(1, 4) = dt;
  gPrime(2, 5) = dt;

  ut = ut * dt;

  gPrime(3, 6) = RbgPrime.row(0) * ut;
  gPrime(4, 6) = RbgPrime.row(1) * ut;
  gPrime(5, 6) = RbgPrime.row(2) * ut;

  ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;

  /////////////////////////////// END STUDENT CODE ////////////////////////////
```
![Scenario 4](./images/scenario_9.gif)

### Step 4: Magnetometer Update Solution ###

The solution of this task is based on section 7.3.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj).

The [UpdateFromMag](./src/QuadEstimatorEKF.cpp#L354-L383) is as shown below,

```
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  hPrime(6) = 1.0;
  zFromX(0) = ekfState(6);

  float magnetometerDiff = magYaw - ekfState(6);
  // Normalize the Yaw
  if ( magnetometerDiff > F_PI ) {
    zFromX(0) += 2.f*F_PI;
  } else if ( magnetometerDiff < -F_PI ) {
    zFromX(0) -= 2.f*F_PI;
  }
  /////////////////////////////// END STUDENT CODE ////////////////////////////
```
The parameter `QYawStd` from [QuadEstimatorEKF.txt](./config/QuadEstimatorEKF.txt) is also tuned so as to approximately capture the magnitude of drift.

![Scenario 5](./images/scenario_10.gif)

### Step 5 & 6: Closed Loop + GPS Update Solution + Custom Controller ###

Perhaps the easiest of the updates to implement, the [UpdateFromGPS](./src/QuadEstimatorEKF.cpp#L321-L352) is,

```
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  for (int i = 0 ; i < 6 ; i++) {
      hPrime(i, i) = 1.0;
  }

  zFromX(0) = ekfState(0);
  zFromX(1) = ekfState(1);
  zFromX(2) = ekfState(2);
  zFromX(3) = ekfState(3);
  zFromX(4) = ekfState(4);
  zFromX(5) = ekfState(5);
  /////////////////////////////// END STUDENT CODE ////////////////////////////
```
Set the `Quad.UseIdealEstimator` to 0 and comment out the lines as shown below,

```
Quad.UseIdealEstimator = 0
#SimIMU.AccelStd = 0,0,0
#SimIMU.GyroStd = 0,0,0
```

On tuning the process noise parameters in [QuadEstimatorEKF](./config/QuadEstimatorEKF.txt), the scenario runs correctly and passes.

For the last part of the project, copy the controller files [QuadControl.cpp](https://github.com/anupamberi/FCND-Controls-CPP/blob/master/src/QuadControl.cpp) and 
[QuadControlParams.txt](https://github.com/anupamberi/FCND-Controls-CPP/blob/master/config/QuadControlParams.txt) from the controls project to the current project.

On the first run, the Quad flies away very rapidly and we need to tune the position and velocity parameters in [QuadControlParams.txt](./config/QuadControlParams.txt).

On successful tuning, the parameters look like below,

```
# Position control gains
kpPosXY = 7
kpPosZ = 30
KiPosZ = 100

# Velocity control gains
kpVelXY = 6
kpVelZ = 20

# Angle control gains
kpBank = 7
kpYaw = 2

# Angle rate gains
kpPQR = 80, 80, 4

```
The scenario using our controller from the previous project is successful,

![Scenario 6](./images/scenario_11.gif)

## References ##

[Estimation for Quadcopter](https://www.overleaf.com/read/vymfngphcccj)