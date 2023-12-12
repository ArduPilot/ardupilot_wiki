.. _chat:

====
Chat
====

OpenAI's ChatGPT Assistants can be used to control an ArduPilot vehicle using the chat module.

.. code:: bash

    module load chat

.. image:: ../../images/chat-module.png

Prerequisites
=============

As of Dec 2023 the Chat module is quite new and requires some manual steps before it can be used.

The Chat module requires an OpenAI API key.  Details on how to get an API key are on the `OpenAI Account setup page  <https://platform.openai.com/docs/quickstart/account-setup>`__.  ArduPilot core developers can request a key in the "dev-team-funding" Discord channel.

If an ArduPilot provided API Key is not used then the OpenAI account holder will need to re-create the "ArduPilot Vehicle Control via MAVLink" assistant which can be done by running the MAVProxy/modules/mavproxy_chat/assistant_setup/setup_assistant.py script.

The pyaudio, wave and openai python packages must be installed manually.

Windows
-------

Open a command prompt and enter the following commands:

- pip install pyaudio
- pip install wave
- pip install openai

Linux/Ubuntu
------------

- sudo apt -y install python3-pyaudio
- pip install wave
- pip install openai

Usage
=====

Normally the chat window will appear as soon as "module load chat" is entered into the MAVProxy console.  If not then please check the console for errors.
