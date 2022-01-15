# Mini Rover RTOS
Mini Rover RTOS is a real-time operation system for the mini-rover.

It use state machine to control the state transit of motors and the rover.

the files are organized as follow:
```
min_rover_rtos
├── config
├── include
├── launch
└── src
```

- `config` folder includes configurations of motors.
- `include` folder includes the head files.
- `launch` folder includes the launch files.
- `src` folder includes the source files.

The motors have 3 states: Idle, Stop and Running.

The rover have 2+n states: Idle, Stop and n Experiment states.
