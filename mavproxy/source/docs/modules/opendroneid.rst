===================
OpenDroneID support
===================

.. code:: bash

    module load opendroneid

When an OpenDroneID module is connected to ArduPilot, this module will send the
applicable GCS data (such as Operator details) to the module, via ArduPilot.

See :ref:`here <dev:opendroneid>` for more details on OpenDroneID.

The module will automatically send the data in the setting below at ``rate_hz``.

=======================   ===================================================  ===============================
Setting                   Description                                          Default
=======================   ===================================================  ===============================
rate_hz                   How often to send the GCS data (Hz)                  0.1
location_rate_hz          How often to send the operator location (Hz)         1
UAS_ID_type               UAS ID Type (0=None, 1=SerialNumber, 2=CAA,          0
                          3=UTM_Assigned, 4=SessionID)                         
UAS_ID                    UAS ID                                               ''
UA_type                   UAS Type (0=None, 1=Aeroplane, 2=Heli or Multi,      0
                          3=Gyroplane, 4=HybridLift, 5=Ornithopter, 6=Glider,  
                          7=Kite, 8=FreeBalloon, 9=CaptiveBalloon, 10=Airshp,  
                          11=Parachute, 12=Rocket, 13=TetheredPowered,         
                          14=GroundObstacle)                                    
description_type          Description Type (0=Text, 1=Emergency,               0
                          2=ExtendedStatus)                                     
description               Description of platform                              ''
area_count                When operating swarm, number of platforms in area    1
area_radius               When operating swarm, radius of operation area       0
area_ceiling              When operating swarm, altitude ceiling of platforms  -1000
area_floor                When operating swarm, altitude floor of platforms    -1000
category_eu               EU Category (0=Undeclared, 1=Open, 2=Specific,       0
                          3=Certified)                                          
class_eu                  EU Class                                             0
classification_type       EU Classification Type (0=Undeclared, 1=EU)          0
operator_location_type    Operator location type (0=Takeoff, 1=LiveGNSS,       0
                          2=Fixed)                                              
operator_id_type          Operator ID type                                     0
operator_id               Operator ID                                          ''
=======================   ===================================================  ===============================