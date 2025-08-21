export EXTRA_BUILD_RUN="groupadd -g $(getent group gpio | cut -d: -f3) gpio -U $(whoami)"
