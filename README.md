# Full Scale WaLTER Simulation Program

## Setup and Installation (Linux)
To begin, make sure that Python 3.12 or later is installed. You will also need either a Logitech gamepad or a PS4 controller connected to the computer with a micro usb cable. The simulation will not run without a controller connected.

Use the following command to check:

``` python --version ```

With the proper Python version installed, navigate to the directory you'd like to store the entire project folder.
For example, you may want the package to be stored in your 'Documents' folder or on your desktop for easy access.

Once in the desired directory, clone the repository using the following commands:

``` git clone git@github.com:dmj17b/Brax_WS.git```

Now that the repository is cloned, navigate into the folder "Brax_WS" that was just created.

Once in the workspace, create a python virtual environment. You may need to install additional Python packages for this to work.

``` python -m venv env ```

"env" will be the name of the Python environment folder that stores all the packages related to this project. You can name this directory whatever you want, but for this tutorial I will be using env.

To activate (source) the Python environment, run the following command:

```source env/bin/activate```

Most terminal environments will use color changes or an added (env) to each line to let you know you've activated the environment properly.

Now that the virtual environment is set up and properly sourced, we can install the required python packages using pip.

To triple check before installing a bunch of dependencies to the wrong place, I like to run

```pip --version```

This command should return a path leading deeper into your 'env' folder, ensuring that the packages are only being installed for this environment, not the entire computer.

Now, you can install the packages needed to run the program.

```pip install -r requirements.txt```

Now all the required dependencies should be installed and you can run the program.

## Running the Simulation

To run the simulation, first make sure you are in the Brax_WS directory, your Python virtual environment is activated, and the gamepad is plugged into the computer. Then you can run:

```python FullScaleSim.py```

This should open a MuJoCo window where you can pan, zoom, and orbit with your mouse. It also allows you to adjust other visual aspects of the simulator. From here, you can pilot the robot according to the prescribed joystick policy.

## Adjusting Motor Models
All of the current motor model parameters can be adjusted in "FullScaleSim.py." The parameters for each actuator are listed at the top of the script as such:
```
hipParams = {
  'Kp': 600,
  'Kd': 80,
  'gear_ratio': 1,
  't_stall': 120,
  'w_no_load': 230*0.1047,
}

kneeParams = {
  'Kp': 600,
  'Kd': 30,
  'gear_ratio': 1,
  't_stall': 100,
  'w_no_load': 230*0.1047,
}

wheelParams = {
  'Kp': 0.5,
  'Kd': 0.3,
  'gear_ratio': 1,
  't_stall': 25,
  'w_no_load': 230*0.1047,
}
```
IMPORTANT NOTES: 
* Gear ratios scale torque and speed, but they DO NOT affect the reflected inertia in the system. This reflected inertia is crucial to the dynamics of the drive train, but the current models should still help us converge on actuator and mass requirements
* Motor models follow conventional speed-torque curve as defined by a stall torque and no-load speed. BLDC motor models will be tested and implemented on other branches.
* There are joint parameters in the MuJoCo XML file that can also be adjusted (models/walter/FullScale.xml)


## Adjusting Body Parameters
Any parameters related to the entire model are described in models/walter/FullScale.xml 

Default classes are created for each linkage and joint at the top of the XML file. 

### Changing Masses
The masses of each linkage are easy to change within the FullScale xml file.





## Troubleshooting
