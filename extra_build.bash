export EXTRA_BUILD_RUN="
groupadd -g $(getent group i2c | cut -d: -f3) i2c -U $(whoami)
groupadd -g $(getent group gpio | cut -d: -f3) gpio -U $(whoami)
usermod -aG audio $(whoami)
usermod -aG video $(whoami)

# Sound setup
cat > /etc/asound.conf <<EOF
# ALSA configuration for:
# 1) Waveshare USB sound out
# 2) I2C mike inputs
#
# Copy this file to /etc/asound.conf to enable.

# I2C mike input
pcm.mike_direct {
  type hw
  card sndrpigooglevoi
  hint.description 'Direct hardware for ear mikes'}

pcm.mike_softvol {
  type softvol
  slave.pcm mike_dsnoop
  control {
    name 'mike_softvol'
    card sndrpigooglevoi
  }
  max_dB 30.0
  min_dB -0.1
  hint.description 'Softmax for ear mikes'
}

ctl.mike_softvol {
  type hw
  card sndrpigooglevoi
}

pcm.mike_dsnoop {
  type dsnoop
  ipc_key 1023
  slave.pcm mike_direct
  hint.description 'DSnoop splitter for ear mikes'
}

pcm.mike {
  type plug
  slave.pcm mike_softvol
  hint.description 'Ear mikes with full conversions'
}
ctl.mike {
  type hw
  card sndrpigooglevoi
}

# Waveshare USB sound output, map to default ALSA setup
pcm.speaker {
  type copy
  slave.pcm 'sysdefault:CARD=Device'
  hint.description 'Waveshare USB Sound Card (alias for default setup)'
}
ctl.speaker {
  type hw
  card Device
}

# defaults

pcm.!default {
  type asym
  playback.pcm 'speaker'
  capture.pcm 'mike'
  hint.description 'Default ALSA device, playback via Waveshare USB, capture via I2C ear mikes'
}

# The below just means that alsamixer with no arguments will control the I2C ear mikes
ctl.!default {
  type hw
  card sndrpigooglevoi
}
EOF
"
#export EXTRA_BUILD_ARGS="--no-cache"
