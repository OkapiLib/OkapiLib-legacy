## EKFFilter

The `EKFFilter` class inherits from `Filter`. It is a very simple implementation of a one-dimensional extended kalman filter. It does not have any internal model and it is only suited for filtering simple signals, such as a robot's velocity.

### Constructor

```c++
//Signature
EKFFilter(const float iQ = 0.0001,  const float iR = ipow(0.2, 2))
```

Process variance is an estimate of how inaccurate the model is. Measurement variance is an estimate of how noisy the sensor is. If you really want to dial in `R` yourself, then you can record unfiltered data of the process held constant at one state (the robot spinning in place at a constant motor power, for example). Then, you can subtract the estimated signal value from the data, leaving only the noise behind.

Parameter | Description
----------|------------
iQ | Process variance
iR | Measurement variance
