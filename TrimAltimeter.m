function altimeter = TrimAltimeter(start_time, end_time, altimeter_in)

  altimeter.utime = TrimTimes(start_time, end_time, altimeter_in.logtime, altimeter_in.utime);
  altimeter.altitude = TrimTimes(start_time, end_time, altimeter_in.logtime, altimeter_in.altitude);
  
  altimeter.logtime = TrimTimes(start_time, end_time, altimeter_in.logtime, altimeter_in.logtime);

end