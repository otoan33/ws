#!/bin/bash

source $(dirname $0)/.env
# WS_NAME=`echo $(basename $(cd $(dirname $0)/../;pwd)) | sed -e 's/-/_/g'`

(
  cd $(dirname $0)
  docker build -t $DOCKER_IMG:latest -f ./Dockerfile .
)
