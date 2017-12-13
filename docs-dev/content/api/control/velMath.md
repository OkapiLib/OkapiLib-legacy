## VelMath

### Constructor

```c++
//Signature
VelMath(const float iticksPerRev, const float iQ = 0.0001, const float iR = ipow(0.2, 2))
VelMath(const VelMathParams& iparams)
```

Parameter | Description
----------|------------
iticksPerRev | Encoder ticks per one revolution
iQ | `EKF` Process variance
iR | `EKF` Measurement variance

### step

```c++
//Signature
float step(const float inewPos)
```

Calculate, filter, and return a new velocity. This need to be called every so many milliseconds (not any faster than 15 ms).

### setTicksPerRev


```c++
//Signature
void setTicksPerRev(const float iTPR)
```

Set a new ticks per rev value.

Parameter | Description
----------|------------
iTPR | Encoder ticks per one revolution

### getOutput

```c++
//Signature
float getOutput() const
```

Return the most recent velocity.

### getDiff

```c++
//Signature
float getDiff() const
```

Return the difference between the last and second to last velocity. Dividing this value by the sample time would give an acceleration.
