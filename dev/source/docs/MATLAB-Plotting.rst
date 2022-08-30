.. _MATLAB-Plotting:

==========================
MATLAB Real Time Plotting
==========================

.. youtube:: m_cmR_wLe5c
    :width: 100%

The same high speed MATLAB UDP connection implemented for SITL can also be used to stream data from ArduPilot to MATLAB. 
This allows real-time plotting and calculations to be done in MATLAB. This is only a one-way link, data cannot be passed 
back to ArduPilot. ArduPilot can be using any of its `Simulation backends <https://youtu.be/YbOZWb8pddk>`__, including 
MATLAB or Simulink provided they are running in a second MATLAB instance. This method can be used to short-cut any analysis 
that would previously require a SITL test flight and log analysis.

To avoid lag the data must be processed by MATLAB fast enough that the input buffer remains empty. A common use case for 
such a tool is to prototype a function. Once the function is working as expected in MATLAB it can be manually translated 
in to C++ and run natively within ArduPilot. The c++ result can then be included in the data streamed to MATLAB to be 
compared with the prototype implementation. 

Minimal changes are required to ArduPilot. Firstly the socket library must be included in the header file associated with 
the cpp of interest. A socket private variable is then defined. For example:

::

  #include <AP_HAL/utility/Socket.h>
  class streaming_example
  {
    private:
    SocketAPM   sock{true};
  };


In the function of interest the data of is then assembled into a structure and sent out over UDP. Note that the structure 
must be aligned, the simplest way to achieve this is to used the same variable types. The IP and port number should be set 
to match those used in MATLAB.

::

  struct interesting_data {
    float data1 = interesting_number1;
    float data2 = interesting_number2;
    float data3 = interesting_number2;
  } data_to_send;

  sock.sendto(&data_to_send, sideof(data_to_send), 127.0.0.1, 9002)

The MATLAB code to receive the data would be as follows, the TCP/UDP/IP Toolbox should be added to the MATLAB path.

::

  % Init the UDP port
  pnet('closeall')
  u = pnet('udpsocket',9002); % port should match that used in AP
  pnet(u,'setreadtimeout',0);

  bytes_read =  3*4; % expecting 3 floats of 4 bytes each

  while true

    % see if there is anything waiting
    in_bytes = pnet(u,'readpacket',bytes_read);

    if in_bytes == 0
        % wait for data
        % you could use this wait to update your plots
        while true
            in_bytes = pnet(u,'readpacket',bytes_read);
            if in_bytes > 0
                break;
            end
        end
    end

    % read in data from AP
    % C++ floats are a MATLAB SINGLE type, were expecting 3, cast to doubles as used by MATLAB natively
    data_received = double(pnet(u,'read',3,'SINGLE','intel'));

    % convert back to AP variables names for consistency
    interesting_number1 = data_received(1);
    interesting_number2 = data_received(2);
    interesting_number3 = data_received(3);

    % do something interesting here

  end




