---
title: Getting started
type: index
---

## Programming a Skills Robot

This tutorial is a step-by-step guide to programming this simple skills robot:

{{< figure src="/images/skillsRobotPicture.jpg#center" >}}

This robot has motors and sensors plugged into the following ports:

Motor Port | Description | Motor Port | Description
-----|-------------|------|------------
1 |                   | 6  | Right middle motor
2 | Left front motor  | 7  | Right bottom motor
3 | Left middle motor | 8  | Lift left motor
4 | Left bottom motor | 9  | Lift right motor
5 | Right top motor   | 10 | 

Digital Port | Description | Digital Port | Description
------------|-------------|-------------|------------
1 | Left encoder top wire     | 7  | 
2 | Left encoder bottom wire  | 8  | 
3 | Right encoder top wire    | 9  | 
4 | Right encoder bottom wire | 10 | 
5 |                           | 11 | 
6 |                           | 12 | 

Analog Port | Description | Analog Port | Description
------------|-------------|-------------|------------
1 | Lift potentiometer | 5 | 
2 |                    | 6 | 
3 |                    | 7 | 
4 |                    | 8 | 

## Project Configuration

If you are creating a new project, the easiest way to get up to speed is to download Okapi's provided [starter project](https://github.com/OkapiLib/StarterProject), which comes pre-configured with everything you need to start coding. Otherwise, you will have to follow the remaining steps in this section:

After installing OkapiLib through the PROS Conductor tool, `okapilib.a` will be copied into `firmware/` and OkapiLib's header files will be copied into `include/`. In order to properly compile your project, you need to modify a few files:

- The `src/` folder contains three files, `auto.c`, `init.c`, and `opcontrol.c`. The `.c` extension of these files needs to be changed to `.cpp` so the compiler knows they contain C++ code.

- The `src/init.cpp` file needs to be changed so it contains the following code which calls a special internal function `__libc_init_array()`:

```c++
#include "main.h"

extern "C" {
  void __libc_init_array();
}

void initializeIO() {
}

void initialize() {
  __libc_init_array();
}
```

- Finally, the `common.mk` file needs to have the `CPPFLAGS` variable modified to contain the flag `-std=c++14` so the compiler knows the correct C++ standard to use:

This line: `CPPFLAGS:=$(CCFLAGS)-fno-exceptions -fno-rtti -felide-constructors`

Should become this: `CPPFLAGS:=$(CCFLAGS) -std=c++14 -fno-exceptions -fno-rtti -felide-constructors`

## Programming User Control

### Drivetrain Setup

Let's start by setting up our motors and sensors configuration. We want to tell Okapi that we have two quadrature encoders and six motors on our chassis.

Our two quad encoders can be written as:

```c++
QuadEncoder leftEnc(1, 2, true), rightEnc(3, 4);
```

Now we can use those quads with our motors to make a model for controlling our chassis:

```c++
SkidSteerModel<3> model({2_m, 3_m, 4_m, 5_m, 6_m, 7_m}, leftEnc, rightEnc);
```

That syntax might look a little weird. Breaking it down further, the template argument to our `SkidSteerModel` specifies the number of motors per side we have. In our case, we have three motors on each side of our chassis, so the template argument value is `3`. Next, the first argument to the `SkidSteerModel` constructor is an array of motors. Each motor is specified using special syntax called a literal, which lets you abbreviate the `Motor` constructor. Writing `2_m` translates into `Motor(2, false)`, which is a non-reversed motor on port two. `2_rm` would translate into `Motor(2, true)`, which is a reversed motor on port two.

It's a good idea to use PID control to have the chassis drive around accurately during autonomous. Okapi can do this using a `ChassisController`, which combines a `ChassisModel` with either PID or Motion Profile control. Let's refactor to use that instead:

 ```c++
 ChassisControllerPid controller(SkidSteerModelParams<3>({2_m,3_m,4_m, 5_m,6_m,7_m}, leftEnc, rightEnc),
                                 PidParams(0.15, 0.05, 0.07),
                                 PidParams(0.02, 0.01, 0));
 ```

Take a look at the first argument to the `ChassisControllerPid` constructor. It looks just like our model from before, except it has a `Parmas` suffix now. Okapi often encapsulates the constructor arguments of classes into their own `Params` class to simplify making complex objects. Notice we do the same thing for the remaining two arguments, `PidParams` specifies how to make a `Pid` controller.

To recap, this is the final state of our chassis setup code:

```c++
QuadEncoder leftEnc(1, 2, true), rightEnc(3, 4);
ChassisControllerPid controller(SkidSteerModelParams<3>({2_m,3_m,4_m, 5_m,6_m,7_m}, leftEnc, rightEnc),
                                PidParams(0.15, 0.05, 0.07),
                                PidParams(0.02, 0.01, 0));
```

### Lift Setup

Okapi provides a tool, called `GenericController`, to associate a group of motors to some closed-loop controller. We want to use this for our lift because all of the lift motors should act like one, so it's easier to think about in that manner. Let's tie our two lift motors together and control them with a special PID controller, called `NsPid`, that moniors velocity and reduces the control signal at low velocity to prevent the system from stalling itself.

```c++
GenericController<2> liftController({8_m, 9_m},
                                    std::make_shared<NsPid>(NsPid(PidParams(0.2, 0.1, 0.1), 
                                                            VelMathParams(360),
                                                            0.5)));
```

Like before, we specify our two motors in an array using motor literals, and we specify the number of motors using the template argument. These two motors will recieve the control signal from the closed-loop controller we specify in the second argument. Notice that because `GenericController` accepts an abstract `ControlObject`, we cannot supply a concrete class directly. Instead, we must make a `std::shared_ptr` to our concrete class.

### Control Code

We'll have a simple state machine that controls the lift's target height:

```c++
const unsigned char liftPot = 1;
constexpr int liftUpTarget = 2570, lift34 = 300, liftDownTarget = 10;
int target = liftUpTarget;

while (1) {
  if (joystickGetDigital(1, 6, JOY_UP))
    target = liftUpTarget;
  else if (joystickGetDigital(1, 6, JOY_DOWN))
    target = liftDownTarget;
  else if (joystickGetDigital(1, 5, JOY_UP))
    target = lift34;
  else if (joystickGetDigital(1, 8, JOY_LEFT)) {
    liftController.flipDisable(); //Turn the controller on or off
    while (joystickGetDigital(1, 8, JOY_LEFT)); //And wait
  }

  liftController.setTarget(target); //Set the new target
  liftController.loop(analogRead(liftPot)); //Loop the controller
}
```
 
 Let's also use arcade control to drive around ourselves:

```c++
 controller.arcade(joystickGetAnalog(1, 2), joystickGetAnalog(1, 1));
```


## Review

 That's all the code we need to drive around and use the lift. Here's our final product:

 ```c++
#include <cmath>
#include "main.h"
#include <chassis/basicChassisController.h>
#include <control/nsPid.h>
#include <control/genericController.h>

using namespace okapi;

void operatorControl() {
  using namespace std; //Needed to get round to compile
  
  QuadEncoder leftEnc(1, 2, true), rightEnc(3, 4);
  ChassisControllerPid controller(SkidSteerModelParams<3>({2_m,3_m,4_m, 5_m,6_m,7_m}, leftEnc, rightEnc),
                                  PidParams(0.15, 0.05, 0.07),
                                  PidParams(0.02, 0.01, 0));

  const unsigned char liftPot = 1;

  GenericController<2> liftController({8_m, 9_m},
                                      std::make_shared<NsPid>(NsPid(PidParams(0.2, 0.1, 0.1),
                                                                    VelMathParams(360),
                                                                    0.5)));

  constexpr int liftUpTarget = 2570, lift34 = 300, liftDownTarget = 10;
  int target = liftUpTarget;

  while (1) {
    if (joystickGetDigital(1, 6, JOY_UP))
      target = liftUpTarget;
    else if (joystickGetDigital(1, 6, JOY_DOWN))
      target = liftDownTarget;
    else if (joystickGetDigital(1, 5, JOY_UP))
      target = lift34;
    else if (joystickGetDigital(1, 8, JOY_LEFT)) {
      liftController.flipDisable();
      while (joystickGetDigital(1, 8, JOY_LEFT));
    }

    liftController.setTarget(target);
    liftController.loop(analogRead(liftPot));

    controller.arcade(joystickGetAnalog(1, 2), joystickGetAnalog(1, 1));
  }
}
```
