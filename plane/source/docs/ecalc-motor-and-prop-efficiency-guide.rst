.. _ecalc-motor-and-prop-efficiency-guide:

=====================================
eCalc Motor and Prop Efficiency Guide
=====================================

A really nice guide from user Dave Smith on how to use eCalc to help you
get the best motor, prop and battery combination for your use.

Go to the eCalc site here:
`PropCalc <http://www.ecalc.ch/motorcalc.php?ecalc>`__

*Disclaimer: I have been in the drone business about two months (since
Christmas). I have noticed that there is a lack of How-To's, unlike
other RC forums. How-To guides are nice when you want to do a simple
search and get results without having to weed through pages and pages of
discussions. I hope to keep blogging my progress to help other noobs.*

Setting up my first drone, I did a search for "motor and prop
efficiency" and came up with a couple of references
to \ `eCalc <http://www.ecalc.ch/>`__. It is an online calculator that
calculates flight times for all sorts of flying drones. I will use the
example of my latest build, a Bixler 1.1. eCalc can be pretty
overwhelming at first, but it becomes a breeze with some practice. 

Weight, Field Elevation and ESC Information
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

First, you need to plug in your motor cooling, field elevation, and
ambient temperature. Notice that when you put in the ESC, you need to
put in the \ **surge or burst** rating:

.. image:: http://api.ning.com/files/gWhc*8EYmk*056RibQYQMXEviNvqyB5ZGnpR0Z1zmqYcI0BmrTMtZCi527vap*sDBt6sPcmcGH2DTQk5k2eHLOfSgKr4T16A/eCalc1.png
    :target:  http://api.ning.com/files/gWhc*8EYmk*056RibQYQMXEviNvqyB5ZGnpR0Z1zmqYcI0BmrTMtZCi527vap*sDBt6sPcmcGH2DTQk5k2eHLOfSgKr4T16A/eCalc1.png

Battery Information
~~~~~~~~~~~~~~~~~~~

Now, put in your battery information. If you cannot find your particular
battery, find a close match in C-rating and capacity and allow the
program to populate the rest of the fields for you. Then, go back to the
drop down and select \ **CUSTOM**. In my example, I chose a 2100 mAh
battery, but went back and changed it to 2200 for more precise flight
times:

.. image:: http://api.ning.com/files/gWhc*8EYmk9l4FEglvwSGqJnQ3PmWGs-3ADj1NEchm15QfjSC1DspdFrq2mMskkwhYkjg13BnXVUBJyxjHi6BlHp5b3EXDaA/eCalc2.png
    :target: ../_images/eCalc2.png

Motor Information
~~~~~~~~~~~~~~~~~

Next, choose your motor, or one from the drop-down that matches pole
count and can size (diameter and can length.) Allow the program to
populate the rest of the fields for you and then go back and change the
fields with information from the manufacturer's website. In my example,
I had to change power (watts) and weight:

.. image:: http://api.ning.com/files/gWhc*8EYmk9bKR4yB0KEmw38gVMoVavotgxs2b8CTcYGnWlJg1jvfFjf7v5S3KWLQ0WQG9H2l2Din5-9yzDGvWEffFskMtV-/eCalc3.png
    :target:  http://api.ning.com/files/gWhc*8EYmk9bKR4yB0KEmw38gVMoVavotgxs2b8CTcYGnWlJg1jvfFjf7v5S3KWLQ0WQG9H2l2Din5-9yzDGvWEffFskMtV-/eCalc3.png

Propeller Information
~~~~~~~~~~~~~~~~~~~~~

Next, select a prop and press \ **CALCULATE**. You want to experiment
with motor and prop combinations that do not give you the red warning
letters:

.. image:: http://api.ning.com/files/gWhc*8EYmk9JGfv7XlQv1U3ancO1aonJ1lZWrz05orWsx21YWX9sIsg10tEl*vn7SUJSHZdOwDUSZ4LD0Xda7HFJ0lk8wL*q/eCalc4.png
    :target:  http://api.ning.com/files/gWhc*8EYmk9JGfv7XlQv1U3ancO1aonJ1lZWrz05orWsx21YWX9sIsg10tEl*vn7SUJSHZdOwDUSZ4LD0Xda7HFJ0lk8wL*q/eCalc4.png

Try Various Propellers
~~~~~~~~~~~~~~~~~~~~~~

After making some changes, the most efficient prop for this motor is a 6
x 3E from APC. Notice the flight times offer you a range, depending on
your flying style:

.. image:: http://api.ning.com/files/gWhc*8EYmk-wH9lw2GStAGUnSZTSyXPeR9a4h5etzgdg5dzjvAsGwmR4cCmXV3hgbpwHtEuEVGKWlx6nn17EZTwEW7kFWWIZ/eCalc5.png
    :target:  http://api.ning.com/files/gWhc*8EYmk-wH9lw2GStAGUnSZTSyXPeR9a4h5etzgdg5dzjvAsGwmR4cCmXV3hgbpwHtEuEVGKWlx6nn17EZTwEW7kFWWIZ/eCalc5.png

Tune For Actual Requirements
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

I just flew this particular combination yesterday, and the "most
efficient" isn't always the best. In my case, the plane flew on the cusp
of stalling with very little reserve thrust. It would literally stand
still in ~20 MPH winds! I changed out for a 6 x 4E prop which gives much
more power while sacrificing very little in flight times:

.. image:: http://api.ning.com/files/gWhc*8EYmk8J8reiqy97JBKUTTQ60akJKaxYiEB2TqaqvAj4aBPUcD*5g8-eLldhuUCx-RbIfnz4I1ozydzbANfZQHXis78s/eCalc6.png
    :target:  http://api.ning.com/files/gWhc*8EYmk8J8reiqy97JBKUTTQ60akJKaxYiEB2TqaqvAj4aBPUcD*5g8-eLldhuUCx-RbIfnz4I1ozydzbANfZQHXis78s/eCalc6.png

I can attest that the accuracy is dead-on with eCalc. At 10-11 minutes
of flight time, my battery is at 50%. I hope this helps someone out on
their journey!

.. |image0| image:: http://api.ning.com/files/gWhc*8EYmk-hLXAf9a2IdhNJp46VlAets0wq2CHQ7kC18dIyrJ*M3*f4KrtQPAqkAYEyNF4PZIKMfXChZgFee-cErMTrkcnp/D28261014002.jpg
    :target:  http://api.ning.com/files/gWhc*8EYmk-hLXAf9a2IdhNJp46VlAets0wq2CHQ7kC18dIyrJ*M3*f4KrtQPAqkAYEyNF4PZIKMfXChZgFee-cErMTrkcnp/D28261014002.jpg