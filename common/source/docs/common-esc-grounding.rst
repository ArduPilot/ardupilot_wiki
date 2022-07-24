.. _common-esc-grounding:

=======================================
ESC Grounding and Wiring Considerations
=======================================

ESC grounding issues are made up 3 forms of ESC signalling/coupling problems, resistive, capacitive and inductive. All three should be considered when building an aircraft.

Inductive
=========

This is the tendency for sudden changes in current to cause large voltage spikes in your system. Inductance in your power system is primarily caused by the size of the loop formed by the positive and negative leads between the ESC and battery. So the longer the cables to the ESC the higher the inductance. However the most important consideration in reducing the inductance is to ensure the positive and negative power wires are kept very close together. This can be done by twisting the two gently together approximately 1 twist every 10 to 20 cm. A 10cm long lead with a large gap between wires will have a much higher inductance than a 1m long lead with minimum spacing between the positive and negative lines.

High inductance is like water hammer in the house pipes. Each time the ESC turns off the voltage across the input of the ESC spikes. In extreme cases this can cause damage to the input capacitor or the FETS. This noise can directly cause signal detection problems or commutation detection problems. These voltage spikes can also cause signal detection problems indirectly by capacitive coupling to your PWM signal line.
(These spikes are absorbed by the large capacitors on the input of your ESC so your broken capacitor wire will make this type of noise more serious)

Capacitive
==========

Capacitive coupling problems are a result of voltage spikes in large power wires or other devices like radio transmitters being coupled to the PWM wires. ESC’s designed with high impedance terminations and don’t have effective filters on the ESC input can be particularly susceptible to this. BLHeli based systems tend to have less filtering due to the higher data rates required. Bi-directional systems make it even more difficult to effectively filter this noise.

Capacitive coupling is minimised by using twisted PWM signal wires as an noise source tends to couple equally to both the ground and signal line. It is also good practice to avoid routing PWM lines next to things like radio transmitters or lidars.

A common source of this coupled lines is the close proximity of power wires connected to ESC’s. This is a difficult problem to avoid as these wires generally must travel in the same space as the power. In this instance the twisted pair, signal and ground, is the only practical option. Other mitigations focus on reducing the magnitude of this noise source (reduce inductance in the power wires and adding additional capacitance at the ESC)

Resistive
=========
This is one of the less well understood coupling mechanisms but is common in many aircraft. The problem stems from the simple fact that the ground or black wires are not 0.0 V. If the negative bullet connector of the battery was at zero volts all other points on the ground line would be at a higher voltage depending on how much current was flowing back towards the battery. It is also important to understand that current will flow back to the battery through all the available ground paths, not just the shortest path.

Most Multirotor aircraft have 4 to 8 ESC’s capable of drawing very high peek currents. This current must flow back to the battery and is divided between the main power ground and the signal ground based on their impedance. 22 AWG wire will have an impedance of something of the order of 48 to 67 Ohm per km while 14 AWG wire will be 8 to 12 ohm. This means for the same length of power and signal wire, for every 7 amps of current draw there will be 6 amps in the 14 AWG and 1 amp in the 22 AWG wire. However all these signal wires go back to the autopilot then through 2 to 4 28ish AWG BEC ground wires. 28 AWG wires have an impedance of 200 to 280 Ohms per km.

The impact of this ground return current in the autopilot BEC wires is an increase in the ground voltage of the autopilot compared to the battery. This often causes current and voltage measurements to read low.

The impact of significant current in the ESC signal ground wire is an increase in the ground reference of the ESC compared to the AutoPilot. This makes the peek voltage of the signal line appear lower. Especially when using 3.3 V signalling this can reduce the signal to a point where it is interpreted as a low state, changing the throttle level or interrupting communication in digital systems (CAN is differential so not impacted as it is not ground referenced).

So what does all that mean from a practical standpoint…

- Keep the negative and positive power wires to any component next to each other (touching) over as much of the rout as possible.
- Avoid loops and large gaps between positive and negative power wires.
- On very long runs to ESC’s additional capacitance may be required at the ESC.
- Use twisted PWM wires to minimise capacitively coupling to the high impedance signal wires.
- If possible, use 5V signalling when long runs are present.
- Avoid routing next to noisy components like other ESC’s, ESC wiring, and radios.
- Where possible add additional ground connections between the autopilot and system ground.
- Ensure all connections, solder joints, bullets, plugs etc are clean and make good contact.