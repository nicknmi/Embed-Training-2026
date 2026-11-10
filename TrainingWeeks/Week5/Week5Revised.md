# Week 5 Revised
Okay, so now you've basically done with all of the trainings, except for two things: DeviceTree and Ozone.

So far in the trainings, you've been working with a mock up of mbedOS (no relation to TR-embed lol, I was fairly confused when I first heard about it), which is our old RTOS. Since this training was written, we have ported to a new RTOS, zephyrOS, but first, what even is an RTOS?

## What is an RTOS? 

RTOS is an acronym that stands for "Real-Time Operation System". Starting with latter half, an operating system is just software that manages the operations (processes, memory, etc.), and some examples include Windows, MacOS, and Linux. Any OS will give you a scheduler, which is basically a set of tools that let you decide when something should execute; however, the power of an RTOS is that our scheduler is deterministic. In Windows for instance, the scheduler optimizes performance, and so you can have variance in the duration of tasks and when they're called. This is fine in desktop application since users generally don't care about variance on the order of milliseconds; however, our controls loop and peripherals definitely do. 

This is where the real-time aspect comes in. An RTOS scheduler differs from a standard OS scheduler in that it's built to ensure deterministic loops, so time-critical tasks are always executed on time and not early or late. You've seen some aspects of this in the training so far, primarily with `Thisthread::sleep_for`, but it goes much farther than that. If you're interested, contact me and we can talk more about scheduling, but honestly you can go your entire time at TR without having to really care that much about it.

### Beyond Scheduling: DeviceTree

Apart from scheduling, RTOS's also give us dedicated tools (application programming interface, or APIs for short) to manage a lot of the lower-level software problems we face. In almost all RTOSes, you'll find a similar set of low-level APIs dedicated to common communication protocols: I2C, SPI, CAN, etc. and amongst these APIs, every RTOS has to decide how they want to manage your I/O. In mbedOS, we did it rather explicitly, by just directly setting pins at runtime logic. However, a consequence of this is that it can be rather messy. For example, let's say that we want to change our wiring since electronics gave us a new PCB, well then we will have to manually find each instance where a pin is called and reassign it; or alternatively, let's just say that we want to reconfigure i2c from running at 100khz to 400khz, since we realized that we can run it on fast mode. In any case, it can be somewhat confusing to find exactly where we configure our I/O and often times seemingly related changes, like changing the IMU pin or the IMU hz can be in entirely different files. That's fine if you've directly worked with the peripheral before, but if you haven't, it can be a headache to dig through the files.

This leads us to the alternative to explicitly configuring our I/O: abstracting it. Namely, we do this in ZephyrOS with the devicetree. 

[TODO: INCLUDE IMAGE HERE]

What devicetree is at its core is a list of structs that dictate the configuration for our I/O, and each one of these structs is called a node. These nodes primarily consist of communication protocol busses (i2c, can, uart, etc.). In our case, since our microcontroller is supported directly by zephyr, it already has a internal `.dtsi` file that has the information for each node, and so we only need to enable/disable them, which ends up being a fairly simple process, since we just write a .overlay file `nucleo_f446re.overlay` to be exact, and that sets the configuration we want. For example, let's examine our i2c1 node. 

```C++
&i2c1 { //The node is labeled with &i2c1, which just lets zephyr know to reference its internal i2c1 setting for our board
    pinctrl-0 = <&i2c1_scl_pb8 &i2c1_sda_pb7>; // pinctrl-0 just sets the pins we want, in this case we're using & again since these are specific pins within the .dtsi
    pinctrl-names = "default"; // This just lets the compiler know we're using the default names and not some alias
    status = "okay"; // status is what enables/disables the node; okay means enabled, "disabled" means disabled
    clock-frequency = <100000>; // Note that each communication protocol can have a different keyword for clockspeed

    scl-gpios = <&gpiob 8 (GPIO_ACTIVE_HIGH | GPIO_OPEN_DRAIN)>; // These two lines set default behavior for the clock and datalines
    sda-gpios = <&gpiob 7 (GPIO_ACTIVE_HIGH | GPIO_OPEN_DRAIN)>;


    imu: ism330@6a { //This is a childnode, note the @6a, which just lets zephyr know what the address is
        compatible = "st,ism330dhcx"; //listing compatible drivers
        reg = <0x6a>; // defining the register again
    };
};
```

Now that we've seen an example of devicetree, let's try to make our own. In testbench/board, write out the devicetree nodes for can1 and can2 

Here are some useful tips: 

- Consult https://pknessness.github.io/nucleo_f446re_pinout/pinout.html for pins
- the pins should be of the form `&can1_["tx" or "rx" here]_[pa##]`
- Set bitrate to `1000000`
- Remember these are structs, don't forget your closing semi-colon


Once you've done that, you're probably wondering how we actually use this devicetree node. Well, in your testbench.cpp, you can have something along the lines of

```C++
static const struct device canbus1 = DEVICE_DT_GET(DT_NODELABEL([your_node]))
```

To break this down a little, the nodelabel is just the name of the node, so for you guys it should be can1 and can2. Also note that not all device tree devices
have the same struct/getter, for example, our IMU example earlier actually has a specific `i2c_dt_spec` and associated getter. 

```C++
static const struct i2c_dt_spec imu_spec = I2C_DT_SPEC_GET(DT_NODELABEL(imu));
```

Once you've done that, go construct your motor. I trust by this point you can dig around and find the constructor somewhere in the code, I only ask that
you don't go looking for it in other testbench.cpp files, and instead find it either in DJImotor, reference the other robot.cpps, (note that the constructor
in zephyr is slightly different than what you saw in week 3 training, so you should go digging in the zephyr codebase for it)

From here, its now time to start writing our startup code in main(). 

If you remember from your week 2, a good habit to have is to have an `init()` fail loudly so you know if something's wrong at start up. Well, the same
idea applies here. Luckily for us, zephyr gives us a handy `device_is_ready(dev_struct)` that takes in a device struct, and returns true if ready, and false
if not. You should go in and write a conditional that either prints out "can1" or whatever device is good, or handles an error properly (hint: what's the safest
thing to do if it doesn't initialize?)

Once we've done that, you're going to want to call

``` C++
DJIMotor::setCanHandlers();
DJIMotor::getCanHandler(CANHandler::CANBUS_1)->registerCallback(
0x201, 0x208, DJIMotor::getCanOneFeedback);
```

and then start a `while(true)` loop, and **remember** to add `k_sleep(100)` at the end, otherwise our loop won't be consistent.

In this while(true) loop, we should call a periodic function. In this case for simplicity, have the motor power increment from 0 to 1000 power, over the course of 5 seconds 
(do the math for your loop to get to that point). Remember to call `DJIMotor::sendValues(true);` after you set our power. Once the 5 seconds are up, the motor should ramp back down to 0, and then repeat. 

Make sure to get your code checked by your embed lead before you test. 

