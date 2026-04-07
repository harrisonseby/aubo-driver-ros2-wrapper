# AUBO Robot Driver ROS2 wrapper

```bash
# Download the correct one
wget http://archive.ubuntu.com/ubuntu/pool/main/p/protobuf/libprotobuf9v5_2.6.1-1.3_amd64.deb

# Extract (no need to install the whole deb)
dpkg-deb -x libprotobuf9v5_2.6.1-1.3_amd64.deb /tmp/proto9

# Check what's inside
ls /tmp/proto9/usr/lib/x86_64-linux-gnu/

# Copy to system lib path
sudo cp /tmp/proto9/usr/lib/x86_64-linux-gnu/libprotobuf.so.9* /usr/local/lib/
sudo ldconfig

# Verify it's found
ldconfig -p | grep "libprotobuf.so.9"
```