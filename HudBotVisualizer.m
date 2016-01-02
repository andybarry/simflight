classdef HudBotVisualizer < BotVisualizer
  % Wraps a bot visualizer into something that also produces LCM messages
  % for a HUD
  
  methods
    
    function obj = HudBotVisualizer(manip,use_contact_shapes)
       if nargin < 2, use_contact_shapes = false; end
       
       obj = obj@BotVisualizer(manip, use_contact_shapes);
      
       try
         obj.pose_msg = mav.pose_t();
         obj.airspeed_msg = mav.indexed_measurement_t();
         obj.u_msg = lcmtypes.lcmt_deltawing_u();
       catch
         disp('Error initializing LCM types.  If you want to use the HUD visualizer, you need to run:')
         disp('    javaaddpath(''/home/abarry/realtime/LCM/LCMtypes.jar'');')
         disp('    javaaddpath(''/home/abarry/pronto-distro/build/share/java/lcmtypes_mav-lcmtypes.jar'');');
         disp('    javaaddpath(''/home/abarry/pronto-distro/build/share/java/lcmtypes_mav_estimator.jar'');');
         
         obj.hud_disabled = true;
         
       end
      
    end

    function draw(obj, t, y)
      
      % call superclass's draw
      draw@BotVisualizer(obj, t, y);
      
      if obj.hud_disabled == true
        return;
      end
      
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
      
      xdot = y(10);
      ydot = y(11);
      zdot = y(12);
      
      x_drake_frame = [0; 0; 0; roll; pitch; yaw; xdot; ydot; zdot; 0; 0; 0;];
      
      % convert to body frame to get forward velocity
      x_body = ConvertToModelFrameFromDrakeWorldFrame(x_drake_frame);
      
      xdot = x_body(7);
      
      
      [elevL, elevR, throttle] = RadiansToServoCommands(elevL, elevR, throttle);
      
      obj.pose_msg.utime = int64(t*1000000);
      
      obj.pose_msg.pos = y(1:3);
      obj.pose_msg.vel = [xdot; 0; y(12)];
      
      obj.pose_msg.orientation = rpy2quat([roll pitch yaw]);
      
      obj.pose_msg.rotation_rate = zeros(3,1);
      
      obj.pose_msg.accel = [0; 0; 9.81];
      
      obj.u_msg.timestamp = int64(t*1000000);
      obj.u_msg.throttle = throttle;
      obj.u_msg.elevonL = elevL;
      obj.u_msg.elevonR = elevR;
      obj.u_msg.is_autonomous = 0;
      obj.u_msg.video_record = 0;
      
      obj.airspeed_msg.utime = int64(t*1000000);
      obj.airspeed_msg.state_utime = int64(t*1000000);
      obj.airspeed_msg.measured_dim = 1;
      
      obj.airspeed_msg.z_indices(1) = 3;
      obj.airspeed_msg.z_effective(1) = xdot;
      
      obj.airspeed_msg.measured_cov_dim = 1;
      obj.airspeed_msg.R_effective(1) = 15;
      
      
      lc = lcm.lcm.LCM.getSingleton();
      
      lc.publish('STATE_ESTIMATOR_POSE', obj.pose_msg);
      
      lc.publish('servo_out', obj.u_msg);
      
      lc.publish('airspeed', obj.airspeed_msg);
      lc.publish('airspeed-unchecked', obj.airspeed_msg);
      
    end
    
  end
  
  methods(Static)
    function InitJava()
      % you must call this when clear java will run successfully
      
      HudBotVisualizer.AddToJavaClasspath('/home/abarry/realtime/LCM/LCMtypes.jar');
      HudBotVisualizer.AddToJavaClasspath('/home/abarry/pronto-distro/build/share/java/lcmtypes_mav-lcmtypes.jar');
      HudBotVisualizer.AddToJavaClasspath('/home/abarry/pronto-distro/build/share/java/lcmtypes_mav_estimator.jar');
      
    end
    
    function found = AddToJavaClasspath(str)
      jcp = javaclasspath;
      found = false;
      
      for i = 1 : length(jcp)
        if ~isempty(strfind(jcp(i), str))
          found = true;
          return;
        end
      end
      
      javaaddpath(str);
    end
    
  end
  
  properties
    pose_msg;
    airspeed_msg;
    u_msg;
    hud_disabled = false;
  end
  
end