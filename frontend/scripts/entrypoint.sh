#!/bin/bash
python3 ./scripts/get_discourse_posts.py
python3 ./scripts/serve.py $SERVICE_PORT
