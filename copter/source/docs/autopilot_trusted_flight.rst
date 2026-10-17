.. _autopilot-assembly-instructions:

=====================================
Autopilot Trusted Flight Instructions
=====================================
Trusted Flight is a set of software services that provide a security layer to operators and manufacturers to conduct flight operations using the mature and well-tested Public Key Infrastructure and commonly available identity and authentication standards. We will cover the concept of “trusted flights” and its open implementation in Ardupilot.

=================
Aerobridge Server
=================
Trusted flight utilizes commonly available Public Key Infrastructure all the transactions detailed here do not need any special software however to automate the storage and issuance of JWT tokens, you can consider the use of the open-source Aerobridge server. It is however, not necessary for use. 

======================================
Autopilot Trusted Flight Instructions
======================================

Pre-requisites
======================================
In order for this to work, you will need to transfer a "root of trust" to the ROMFS on the drone. In this case, the domain of the auth server which issues the JWT token validated by Let's Encrypt and we install the Let's encrypt root certificate on the drone.

Steps for testing trusted flights locally
=========================================

1. Move to scripts directory:

  ``cd Tools/scripts/AP_AerobridgeTrustedFlight/``

2. Generate self signed root certificate in ``/tmp/trusted_flight_test``
   directory:

  ``./generate_root_cert.py /tmp/trusted_flight_test``

3. Generate certificate chain and JWT token for trusted flights with 3
   intermediate CAs (could be any number of intermediate CAs):

   ``./generate_ca_chain_and_token.py /tmp/trusted_flight_test 3``

4. Validate generate certs and tokens. This step is not essential for
   trusted flight workflow and only performs initially sanity checks on
   generated artifacts.

   ``./validate_ca_chain_and_token.py /tmp/trusted_flight_test``

5. Build and test Trusted Flights in SITL (ArduCopter)

   -  Copy ``/tmp/trusted_flight_test/ca_chain.crt`` and ``/tmp/trusted_flight_test/token`` to ``ArduCopter/trusted_flight``  

    ``cd /path/to/ardupilot/root/ArduCopter``

    ``mkdir trusted_flight``

    ``cp /tmp/trusted_flight_test/ca_chain.crt trusted_flight/``

    ``cp /tmp/trusted_flight_test/token trusted_flight/``

   -  Build and run SITL

      ``../Tools/autotest/sim_vehicle.py --console --map --osd --trusted-flight-issuer=leaf.cname --trusted-flight-root-certificate=/tmp/trusted_flight_test/root_ca/certificate.crt``

   -  Arm via console ``arm throttle``

6. Build and test Trusted Flights on CubeOrange

   -  Build and upload firmware to CubeOrange

     ``cd /path/to/ardupilot/root``

     ``./waf clean``

     ``./waf configure --board CubeOrange --trusted-flight-issuer=leaf.cname --trusted-flight-root-certificate=/tmp/trusted_flight_test/root_ca/certificate.crt``

     ``./waf --targets bin/arducopter --upload``

   -  Upload ``/tmp/trusted_flight_test/ca_chain.crt`` and
      ``/tmp/trusted_flight_test/token`` to ``/APM/trusted_flight``
      directory on CubeOrange

   -  Arm the vehicle.


More information
================
- For more information re the management server see blog post here: https://blog.openskies.sh/articles/aerobridge-trusted-flight/
- For technical introduction see these two blog posts: 
    - https://medium.com/@rhythm8/a-journey-to-offline-jwt-authentication-ebc7859f0246 
    - https://medium.com/@rhythm8/using-certificate-chain-of-trust-to-verify-jwt-offline-ab3d4c3f0322