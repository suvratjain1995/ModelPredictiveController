# WriteUp #

The model is a Kinematic model. It Ignores interactions between tires and groud and friction components.

The model Equations are as follows:- 

```c
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

Where:

- `x, y` : Car's position.
- `psi` : Car's heading direction.
- `v` : Car's velocity.
- `cte` : Cross-track error.
- `epsi` : Orientation error.

Those values are considered the state of the model. `Lf` is the distance between the center of mass of the car and the front wheels (this is provided by Udacity's seed project). The other two values are the model output:

- `a` : Car's acceleration (throttle).
- `delta` : Steering angle.

The objective is to find the acceleration (`a`) and the steering angle(`delta`) in the way it will minimize an objective function that is the combination of different factors:

- Square sum of `cte` and `epsi`.
- Square sum of `delta` and `throttle`.
- Square sum of difference between reference velocity and actual velocity.
- Square sum of difference between `delta` at `t+1` and `t`.
- Sqare sum of difference between `throttle` at `t+1` and `t`.

Weights of these error were manually tuned to make sure the car could run around the track properly.

### Timestep Length and Elapsed Duration (N & dt)

The number of points(`N`) and the time interval(`dt`) define the prediction horizon. The number of points impacts the controller performance as well. Increasing the value of N can lead to controller being wobbly to adapt to future changes.Low value of N can lead to car travelling slower.The best value after manual tuning the parameters for N and dt were **N 10,0.1 seconds**.The value dt tells us the time stamp between each actutaion.

### Polynomial Fitting and MPC Preprocessing

The waypoints provided by the simulator are transformed to the car coordinate system and are then fed to find the polyfit for 3rd degree polynomial.Chosing value higher than 3 can lead the system to overfit.
```c
          Eigen::VectorXd xvals(ptsx.size());
          Eigen::VectorXd yvals(ptsy.size());
          for (int i = 0; i < ptsx.size(); i++) {
            double delta_x = ptsx[i] - px;
            double delta_y = ptsy[i] - py;
            xvals[i]=(delta_x * cos(-psi) - delta_y * sin(-psi));
            yvals[i]=(delta_x * sin(-psi) + delta_y * cos(-psi));
          }

          auto coeffs=polyfit(xvals,yvals,3);
```

### Model Predictive Control with Latency

To handle actuator latency, the state values are calculated using the model and the delay interval.
Considering intial state of x=y=psi=0.0.
These values are used instead of the initial one.
```c
     double latecy = 100.0;
          double delay  = latecy/1000.0;
          double Lf = 2.67;

          double x_ = 0.0+v*cos(0.0)*delay;
          double y_ = 0.0+v*sin(0.0)*delay;
          double v_ = v+current_throttle*delay;
          double psi_ = 0.0 - (v * current_steering * delay / Lf);
          double cte_ = cte + ( v * sin(epsi) * delay );
          double epsi_ = epsi - ( v * atan(coeffs[1]) * delay / Lf);

```