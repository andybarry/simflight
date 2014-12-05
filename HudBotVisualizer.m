classdef HudBotVisualizer < BotVisualizer
  % Wraps a bot visualizer into something that also produces LCM messages
  % for a HUD
  
  methods
    
    function obj = HudBotVisualizer(manip,use_contact_shapes)
       if nargin < 2, use_contact_shapes = false; end
       
       obj = obj@BotVisualizer(manip, use_contact_shapes);
      
       javaaddpath('/home/abarry/realtime/LCM/LCMtypes.jar');
       javaaddpath('/home/abarry/Fixie/build/share/java/lcmtypes_mav-lcmtypes.jar');
      
       obj.pose_msg = mav.pose_t();
       obj.baro_msg = lcmtypes.lcmt_baro_airspeed();
       obj.u_msg = lcmtypes.lcmt_deltawing_u();
       
      
    end
    
    function draw(obj, t, y)
      
      % call superclass's draw
      draw@BotVisualizer(obj, t, y);
      
      % send data to LCM HUD
      
      if isempty(obj.pose_msg)
        return;
      end
      
      roll = y(4);
      pitch = y(5);
      yaw = y(6);
      

      throttle = y(7);
      elevL = y(8);
      elevR = y(9);
      
      
      [elevL, elevR, throttle] = RadiansToServoCommands(elevL, elevR, throttle);
      
      obj.pose_msg.utime = int64(t*1000000);
      
      obj.pose_msg.pos = y(1:3);
      obj.pose_msg.vel = y(10:12);
      
      obj.pose_msg.orientation = rpy2quat([roll pitch yaw]);
      
      obj.pose_msg.rotation_rate = zeros(3,1);
      
      obj.pose_msg.accel = [0; 0; 9.81];
      
      obj.u_msg.timestamp = int64(t*1000000);
      obj.u_msg.throttle = throttle;
      obj.u_msg.elevonL = elevL;
      obj.u_msg.elevonR = elevR;
      obj.u_msg.is_autonomous = 0;
      obj.u_msg.video_record = 0;
      
      obj.baro_msg.utime = int64(t*1000000);
      obj.baro_msg.airspeed = y(7);
      obj.baro_msg.baro_altitude = y(3);
      obj.baro_msg.temperature = 0;
      
      
      lc = lcm.lcm.LCM.getSingleton();
      
      lc.publish('STATE_ESTIMATOR_POSE', obj.pose_msg);
      
      lc.publish('servo_out', obj.u_msg);
      
      lc.publish('baro-airspeed', obj.baro_msg);
      
    end
    
  end
  
  properties
    pose_msg;
    baro_msg;
    u_msg;
  end
  
end