# Servo_Controller
This is the documentation for the servo controller module. To explore the PCB that goes along with this code, go to the hardware folder of the github below.
To learn how the project works go to the main README in the github. This README is just for the code and for how to configure it.

[ServoControl Github](https://github.com/ljohanon06/ServoControl/)

---

## Coding
This is how to code the servo_controller. To see the code documentation involving all the arguments and return values of the functions,
please view the html documentation at the link below.

[Servo_Controller Documentation](https://ljohanon06.github.io/ServoControl/servo__controller_8h.html)

### Config Struct
When coding, first initialize the config struct as shown below, filling in your gpio numbers and servo precision. Gpio numbers must be capable of output and not be used for anything else.
```c
servo_controller_config_t servo_cfg = {
    .clk_gpio = GPIO_NUM_4,
    .pulse_gpio = GPIO_NUM_5,
    .servo1_gpio = GPIO_NUM_6,
    .servo2_gpio = GPIO_NUM_7,
    .serv_pres = SERVO_PRECISION_500
};
```

### Intialization and Starting
Initialize the servo according to the code below, servo angles will be initialized to 0 degrees. After the servo controller is initialized, you must start the controller before the servos begin to move, you can set the servo angles while the servos aren't started.
```c
servo_controller_handle_t handle = NULL;
servo_initialize_controller(&servo_cfg, &handle);
servo_start(handle);
```

### Stopping and Deleting
When finished using the servo controller, the controller can be stopped and then deleted. Please stop the controller before deleting it. Servos can be started again after being stopped, but while being stopped, the servos will not hold their position under load.
```c
servo_stop(handle);
servo_remove_controller(handle);
```

### Setting Servo Angles
Servo angles can either be set individually or as a whole. Servo numbers are also 0-indexed, meaning the first servo is referred to as servo 0, and the last servo servo is referred to as servo 31. Servos can be set between 0 and 180 degrees.
```c
servo_set_all(handle,90);
servo_set_servo(handle,31,180);
```
---

## Importing


