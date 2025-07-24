import struct
with open('/proc/device-tree/system/linux,revision', 'rb') as f:
   revision = struct.unpack('>I', f.read(4))[0]
print(hex(revision))

