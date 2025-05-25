#!/usr/bin/env bash

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
EXEC_PATH=$PWD

cd $ROOT_DIR


USERNAME=$(whoami)
SSH_PATH="/home/${USERNAME}/.ssh"

# Проверка существования папки
if [ -d "$SSH_PATH" ]; then
  cp -r ${SSH_PATH} $EXEC_PATH/..
else
  echo "Папка ${SSH_PATH} не существует."
  exit 1
fi

if [[ $1 = "--nvidia" ]] || [[ $1 = "-n" ]]
  then
    docker build -t ros-humble-img -f $ROOT_DIR/docker/Dockerfile $ROOT_DIR \
                                  --network=host \
                                  --build-arg from=nvidia/cuda:12.1.0-base-ubuntu22.04 \
                                  --build-arg USERNAME=$(whoami)


else
    echo "[!] If you use nvidia gpu, please rebuild with -n or --nvidia argument"
    docker build -t ros-humble-img -f $ROOT_DIR/docker/Dockerfile $ROOT_DIR \
                                  --network=host \
                                  --build-arg from=ubuntu:22.04 \
                                  --build-arg USERNAME=$(whoami)
fi

cd $EXEC_PATH

# bash requirements/workspace_building.sh
