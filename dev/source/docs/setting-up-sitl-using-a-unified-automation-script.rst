Automating SITL Launch with a Unified Shell Script
==================================================

This guide provides an optional streamlined workflow for running SITL, based on common setup pain points.

Setting up a Software In The Loop (SITL) environment can be repetitive, requiring multiple commands to navigate directories, export paths, and launch various windows (Map, Console, GCS). This guide provides a robust shell script to automate this workflow into a single interactive command.

The Problem it Solves
---------------------
* **Command Fatigue:** Eliminates the need to manually ``cd`` into vehicle directories.
* **Path Management:** Automatically handles the ``Tools/autotest`` and ``.local/bin`` environment paths without cluttering your ``.bashrc``.
* **Environment Conflicts:** Uses a smart search to find the ArduPilot source code, ensuring the script works regardless of where you cloned the repository.
* **One-Click Setup:** Launches the SITL instance, MAVProxy Map, and Console simultaneously.
* **Parameter Recovery:** Includes a "Wipe" option to reset virtual EEPROM if parameters become corrupted.

Prerequisites
-------------
* **Linux Environment:** Compatible with Ubuntu, Linux Mint, or WSL2.
* **ArduPilot Installation:** The source code must be cloned and environment dependencies installed via ``install-prereqs-ubuntu.sh``. 
* **Directory Preference:** While the script is capable of a system-wide search, it is **optimized for installations where the ``ardupilot/`` directory is located within the user's Home (``$HOME``) directory.**
* **Compiled Firmware:** You must have compiled the target vehicle (e.g., ``./waf copter``) at least once prior to running the simulation.

Creating the Script
-------------------
1. Open your terminal and navigate to your desired working directory.
2. Initialize and open the script file using the ``nano`` text editor:

   .. code-block:: bash

      nano launch_sitl.sh

3. **Paste** the code block provided below into the terminal.
4. **Saving and Exiting nano (CRITICAL):**
   To ensure the file is saved correctly without creating a ``.save`` error:
   * Press ``Ctrl + O`` (Write Out).
   * Press ``Enter`` to confirm the filename (``launch_sitl.sh``).
   * Press ``Ctrl + X`` to exit the editor and return to the prompt.

   .. code-block:: bash

      #!/bin/bash

      # --- 1. UNIVERSAL DIRECTORY FINDER ---
      echo "🔍 Searching for ArduPilot source (this may take a second)..."

      # Search Home first (fast), then search the whole system if not found.
      AP_DIR=$(find $HOME -maxdepth 3 -not -path '*/.*' -type d -name "ardupilot" | head -n 1)

      if [ -z "$AP_DIR" ]; then
          # Fallback: search common mount points/root if not in Home
          AP_DIR=$(find / -maxdepth 4 -not -path '*/.*' -type d -name "ardupilot" 2>/dev/null | head -n 1)
      fi

      if [ -z "$AP_DIR" ] || [ ! -f "$AP_DIR/Tools/autotest/sim_vehicle.py" ]; then
          echo "❌ Error: Could not find the real ArduPilot source folder anywhere."
          echo "Please ensure ArduPilot is installed and the folder is named 'ardupilot'."
          exit 1
      fi

      # --- 2. SESSION ENVIRONMENT SETUP ---
      export PATH="$PATH:$AP_DIR/Tools/autotest"
      export PYTHONPATH="$PYTHONPATH:$AP_DIR"
      export PATH="$PATH:$HOME/.local/bin" 

      echo "✅ ArduPilot Source Found: $AP_DIR"
      echo "-------------------------------------------"
      echo "Select Launch Mode:"
      echo "1) Standard (Map + Console)"
      echo "2) Terminal Only (Lightweight)"
      echo "3) Reset & Launch (Wipe Parameters)"
      echo "4) Exit"
      echo "-------------------------------------------"
      read -p "Enter choice [1-4]: " choice

      VEHICLE_DIR="$AP_DIR/ArduCopter"

      case $choice in
          1)
              echo "🚀 Launching SITL + Map + Console..."
              gnome-terminal -- bash -c "cd $VEHICLE_DIR && sim_vehicle.py -v ArduCopter --map --console; exec bash"
              ;;
          2)
              echo "🚀 Launching Terminal-only SITL..."
              cd "$VEHICLE_DIR" && sim_vehicle.py -v ArduCopter
              ;;
          3)
              echo "🧹 Wiping parameters and launching..."
              gnome-terminal -- bash -c "cd $VEHICLE_DIR && sim_vehicle.py -v ArduCopter -w --map --console; exec bash"
              ;;
          4)
              exit 0
              ;;
          *)
              echo "Invalid option."
              ;;
      esac

Usage
-----
To make the script executable, apply the following permissions:

.. code-block:: bash

   chmod +x launch_sitl.sh

Run the script whenever you wish to initiate a simulation session:

.. code-block:: bash

   ./launch_sitl.sh

Troubleshooting
---------------
* **Command Not Found:** Ensure you have completed the ArduPilot environment setup by running ``install-prereqs-ubuntu.sh``.
* **WSL2 Users:** If you are operating without a configured X-Server (GUI), please utilize **Option 2** for a terminal-only simulation.
* **Terminal Emulator Issues:** This script uses ``gnome-terminal``. If you are using a different desktop environment, you may need to install ``gnome-terminal`` or modify the script to call your specific emulator (e.g., ``xfce4-terminal``).
