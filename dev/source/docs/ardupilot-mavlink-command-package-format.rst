.. _ardupilot-mavlink-command-package-format:

========================================
ArduPilot Mission Command Package Format
========================================

ArduPilot's Mission command list is stored in eeprom with each command requiring 14 bytes arranged as follows:

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Byte #</th>
   <th>Address</th>
   <th>Data type</th>
   <th>Function</th>
   </tr>
   <tr>
   <td>0</td>
   <td>0x00</td>
   <td>byte</td>
   <td>Command ID</td>
   </tr>
   <tr>
   <td>1</td>
   <td>0x01</td>
   <td>byte</td>
   <td>Options</td>
   </tr>
   <tr>
   <td>2</td>
   <td>0x02</td>
   <td>byte</td>
   <td>Parameter 1</td>
   </tr>
   <tr>
   <td>3</td>
   <td></td>
   <td>long</td>
   <td>Parameter 2</td>
   </tr>
   <tr>
   <td>4</td>
   <td>0x04</td>
   <td>..</td>
   </tr>
   <tr>
   <td>5</td>
   <td>0x05</td>
   <td>..</td>
   </tr>
   <tr>
   <td>6</td>
   <td>0x06</td>
   <td>..</td>
   </tr>
   <tr>
   <td>7</td>
   <td>0x07</td>
   <td>long</td>
   <td>Parameter 3</td>
   </tr>
   <tr>
   <td>8</td>
   <td>0x08</td>
   <td>..</td>
   </tr>
   <tr>
   <td>9</td>
   <td>0x09</td>
   <td>..</td>
   </tr>
   <tr>
   <td>10</td>
   <td>0x0A</td>
   <td>..</td>
   </tr>
   <tr>
   <td>11</td>
   <td>0x0B</td>
   <td>long</td>
   <td>Parameter 4</td>
   </tr>
   <tr>
   <td>12</td>
   <td>0x0C</td>
   <td>..</td>
   </tr>
   <tr>
   <td>13</td>
   <td>0x0D</td>
   <td>..</td>
   </tr>
   <tr>
   <td>14</td>
   <td>0x0E</td>
   <td>..</td>
   </tr>
   </tbody>
   </table>
