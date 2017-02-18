simflight
=========

Simulation environment for flight systems, based on [drake](http://drake.mit.edu).

### Dependancies
* MATLAB
* [Drake](http://drake.mit.edu)


### Examples

 - Example logs can be downloaded here:
  - For system identification: http://abarry.org/files/logs/odroid-gps2-2015-09-17-field-test.zip
  - Obstacle avoidance: http://abarry.org/files/logs/2015-with-obstacles-mat-and-LCM-logs.zip
  - Datafiles: https://drive.google.com/drive/folders/0B504R2Bs0QpzcVlBRXhsREVLMFE


#### Running system identification using logs

- Use ````do_pem.m```` for system identification of the model ````tbsc_model.m````:

````matlab
% Load dependancies
>> addpath('<path to Drake, for example: /home/user/drake/build/matlab/>')
>> addpath_drake

% Run PEM system identification
% Edit do_pem.m and fill in:
%   logfile_path
%   logfile_name

% Execute
>> do_pem

````

#### Generating a trajectory library
````matlab
>> BuildTrajectoryLibrary
````
#### Use a trajectory library
````
>> lib.ListTrajectories

0: TI-straight-R-200
1: TI-climb-R-200
2: takeoff-no-throttle
3: TI-left-turn1-R-200
4: TI-right-turn1-R-200
5: TI-aggressive-left-turn2-R-200
6: TI-aggressive-right-turn2-R-200

>> lib.DrawTrajectories

>> lib.SimulateTrajectory(0)

    Simulating: "TI-straight-R-200" for 1 second(s) with default initial conditions...
````
