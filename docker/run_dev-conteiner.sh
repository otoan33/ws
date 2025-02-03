#!/bin/bash

source $(dirname $0)/.env

rocker --user --home \
  --name test-dev \
  --volume /etc/group /etc/passwd /etc/shadow \
  --x11 $DOCKER_IMG:latest
