require 'serialport'

sp = SerialPort.new('/dev/tty.SLAB_USBtoUART', 9600, 8, 1, SerialPort::NONE)

while (true) do
  sp.write('x')
  sp.write('gps_aprs_lat')
  sp.write(", ")
  sp.write('gps_aprs_lon')
  sp.write(", ")

  sp.write('0')
  sp.write(", ")
  sp.write('0')
  sp.write(", ")

  sp.write('100')
  sp.write(", ")
  sp.write('0')
  sp.write(", ")
  sp.write(gps_course)
  sp.write(", ")

  sp.write(ahrs_heading)
  sp.write(", ")
  sp.write(current_roll)
  sp.write(", ")
  sp.write(heel_adjust)
  sp.write(", ")
  sp.write(wind)
  sp.write(", ")
  sp.write(wp_heading)
  sp.write(", ")
  sp.write(wp_distance)
  sp.write(", ")
  sp.write(current_rudder)
  sp.write(", ")
  sp.write(current_winch)
  sp.write(", ")
  sp.write(voltage)
  sp.write(", ")
  sp.write(cycle)
  sp.write('d');
end

sp.close