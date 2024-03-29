## XDriveModel

The `XDriveModel` class inherits from `ChassisModel` and takes a template parameter `size_t motorsPerCorner` (the number of motors per each of the four corners of the chassis). It is a model for an x-drive.

### Constructor

```c++
//Signature
XDriveModelParams(const std::array<unsigned char, motorsPerCorner * 4>& imotorList, const QuadEncoder& ileftEnc, const QuadEncoder& irightEnc):
XDriveModelParams(const std::array<unsigned char, motorsPerCorner * 4>& imotorList, const IME& ileftIME, const IME& irightIME)
XDriveModel(const XDriveModelParams<motorsPerCorner>& iparams)
XDriveModel(const XDriveModel<motorsPerCorner>& other)

//Construct an XDriveModel with four motors (one per corner) and two encoders
//Top left motor is port 1, top right motor is port 2
//Bottom right motor is port 3, bottom left motor is port 4
//Right side encoder is reversed because it is a mirror of the left side
XDriveModel<1> foo({1, 2, 3, 4}, QuadEncoder(1, 2, false), QuadEncoder(3, 4, true));

//Construct an XDriveModel with eight motors (two per corner) and two encoders
//Top left motors are ports 1 and 2, top right motors are ports 3 and 4
//Bottom right motors are ports 5 and 6, bottom left motors are ports 7 and 8
//Right side encoder is reversed because it is a mirror of the left side
XDriveModel<2> foo({1, 2, 3, 4, 5, 6, 7, 8}, QuadEncoder(1, 2, false), QuadEncoder(3, 4, true));
```

Parameter | Description
----------|------------
imotorList | The chassis motors for the drive in the clockwise format, `{top left motors, top right motors, bottom right motors, bottom left motors}`
ileftEnc | The quadrature encoder for the left side
irightEnc | The quadrature encoder for the right side
ileftIME | The IME for the left side
irightIME | The IME for the right side

### xArcade

```c++
//Signature
void xArcade(const int verticalVal, const int horizontalVal, const int rotateVal)
```

Specifically for an x-drive, power the motors like arcade drive with a channel for rotation.

Parameter | Description
----------|------------
verticalVal | Motor power for the vertical component of movement
horizontalVal | Motor power for the horizontal component of movement
rotateVal | Motor power for rotation
