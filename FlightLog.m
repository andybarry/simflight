classdef FlightLog
  properties
    date;
    log_number;
    stabilization_trajectory = 0;
    hostname;
    trajlib_path;
    trajlib;
    name = 'field-test';
    
    % data
    MAV_STATE_EST_INITIALIZER
    MAV_STATE_EST_INIT_COMPLETE
    STATE_ESTIMATOR_STATE
    airspeed
    airspeed_unchecked
    altimeter
    altitude_reset
    arm_for_takeoff
    battery
    beep
    cpu
    est
    git_status_AAAZZZA
    git_status_odroid_cam
    git_status_odroid_gps
    gps
    imu
    log_info_odroid_cam
    log_info_odroid_gps
    rad_to_servo
    rc_switch_action
    rc_trajectory_commands
    servo_minmaxtrim
    servo_to_rad
    sideslip
    state_machine_state
    stereo
    stereo_control
    stereo_monitor
    tvlqr
    tvlqr_out
    u
    log
    
    
    t_start;
    t_end;
    
  end
  
  methods
    function obj = FlightLog(date, plane_number, log_number, trajlib)
      obj.date = date;
      obj.log_number = log_number;
      obj.hostname = ['odroid-gps' num2str(plane_number)];
      obj.trajlib = trajlib;
      
      % load the data
      %disp(['Loading ' obj.trajlib_path '...']);
      %load(obj.trajlib_path);
      %obj.trajlib = lib;
      
      dir = [obj.date '-' obj.name '/' obj.hostname '/'];
      filename = ['lcmlog_' strrep(obj.date, '-', '_') '_' obj.log_number '.mat'];


      dir_prefix = '/home/abarry/rlg/logs/';
      dir = [ dir_prefix dir ];

      addpath('/home/abarry/realtime/scripts/logs');

      disp(['Loading ' filename '...']);
      loadDeltawing
      
      % save data
      if (exist('MAV_STATE_EST_INITIALIZER', 'var'))
        obj.MAV_STATE_EST_INITIALIZER = MAV_STATE_EST_INITIALIZER;
      end
      if (exist('MAV_STATE_EST_INIT_COMPLETE', 'var'))
        obj.MAV_STATE_EST_INIT_COMPLETE = MAV_STATE_EST_INIT_COMPLETE;
      end
      if (exist('STATE_ESTIMATOR_STATE', 'var'))
        obj.STATE_ESTIMATOR_STATE = STATE_ESTIMATOR_STATE;
      end
      obj.airspeed = airspeed2;
      obj.airspeed_unchecked = airspeed_unchecked;
      obj.altimeter = altimeter;
      obj.altitude_reset = altitude_reset;
      obj.arm_for_takeoff = arm_for_takeoff;
      obj.battery = battery;
      obj.beep = beep;
      obj.cpu = cpu;
      obj.est = est;
      obj.git_status_AAAZZZA = git_status_AAAZZZA;
      if (exist('git_status_odroid_cam1', 'var'))
        obj.git_status_odroid_cam = git_status_odroid_cam1;
        obj.git_status_odroid_gps = git_status_odroid_gps1;
      end
      if (exist('git_status_odroid_cam2', 'var'))
        obj.git_status_odroid_cam = git_status_odroid_cam2;
        obj.git_status_odroid_gps = git_status_odroid_gps2;
      end
      if (exist('git_status_odroid_cam3', 'var'))
        obj.git_status_odroid_cam = git_status_odroid_cam3;
        obj.git_status_odroid_gps = git_status_odroid_gps3;
      end
      obj.gps = gps;
      obj.imu = imu;
      if (exist('log_info_odroid_cam1', 'var'))
        obj.log_info_odroid_cam = log_info_odroid_cam1;
        obj.log_info_odroid_gps = log_info_odroid_gps1;
      end
      if (exist('log_info_odroid_cam2', 'var'))
        obj.log_info_odroid_cam = log_info_odroid_cam2;
        obj.log_info_odroid_gps = log_info_odroid_gps2;
      end
      if (exist('log_info_odroid_cam3', 'var'))
        obj.log_info_odroid_cam = log_info_odroid_cam3;
        obj.log_info_odroid_gps = log_info_odroid_gps3;
      end
      obj.log_number = log_number;
      obj.rad_to_servo = rad_to_servo;
      obj.rc_switch_action = rc_switch_action;
      obj.rc_trajectory_commands = rc_trajectory_commands;
      obj.servo_minmaxtrim = servo_minmaxtrim;
      obj.servo_to_rad = servo_to_rad;
      obj.sideslip = sideslip;
      obj.state_machine_state = state_machine_state;
      obj.stereo = stereo2;
      if (exist('stereo_control', 'var'))
        obj.stereo_control = stereo_control;
      end
      obj.stereo_monitor = stereo_monitor;
      obj.tvlqr = tvlqr;
      obj.tvlqr_out = tvlqr_out;
      obj.u = u;
      obj.log = log42;
      
      % compute start and end for the flight
      [obj.t_start, obj.t_end] = FindAutonomousFlight(obj);
      
    end
    
  end

end