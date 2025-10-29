export EXTRA_BUILD_RUN="
groupadd -g $(getent group i2c | cut -d: -f3) i2c -U $(whoami)
groupadd -g $(getent group gpio | cut -d: -f3) gpio -U $(whoami)
usermod -aG audio $(whoami)
"
#export EXTRA_BUILD_ARGS="--no-cache"
