#! /usr/bin/bash

cd $1 && \
uvicorn websocket-server:app --reload
