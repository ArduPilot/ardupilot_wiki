=============================
Direct Comms with SPI/I2C bus
=============================

.. code:: bash

    module load devop
    
This module allows for development operations. Specifically the ability to 
read and write directly to the SPI and I2C buses.

I2C
===

To write to the I2C bus

.. code:: bash

    devop write i2c name bus address regstart count <bytes>
    
Where name= device name, bus=i2c bus #, address=device address, regstart=register to 
start at, count=number of bytes to write, <bytes>=the bytes to send.

To read from the I2C bus

.. code:: bash

    devop read i2c name bus address regstart count
    
Where name= device name, bus=i2c bus #, address=device address, regstart=register to 
start at, count=number of bytes to write.

SPI
===

To write to the SPI bus

.. code:: bash

    devop write spi name bus address regstart count <bytes>
    
Where name= device name, bus=spi bus #, address=device address, regstart=register to 
start at, count=number of bytes to write, <bytes>=the bytes to send.

To read from the SPI bus

.. code:: bash

    devop read spi name bus address regstart count
    
Where name= device name, bus=spi bus #, address=device address, regstart=register to 
start at, count=number of bytes to write.


