#!/bin/bash

/ros_entrypoint.sh

UNAME=`whoami`
EXEC_CMD=""
if [ $UNAME != root ];then
  sudo chown $UNAME:$UNAME /home/$UNAME
else
  UNAME=rosuser
  USER_ID=60001
  GROUP_ID=60001
  echo "Start as \"$UNAME\"(UID: $USER_ID, GID: $GROUP_ID)"
  useradd -u $USER_ID -o -m $UNAME
  groupmod -g $GROUP_ID $UNAME
  export HOME=/home/$UNAME
  EXEC_CMD="`which gosu` $UNAME"
fi

cat << '_EOS_' > $HOME/.bashrc
# /etc/profile works only in `bash -l`
source /usr/share/bash-completion/bash_completion

if [ -f /.dockerenv ]; then
  source "/opt/ros/$ROS_DISTRO/setup.bash"
fi
_EOS_

exec $EXEC_CMD "$@"
