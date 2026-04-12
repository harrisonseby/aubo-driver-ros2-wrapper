# AUBO Robot Driver (ROS2 Wrapper)

A ROS2 `ros2_control` hardware interface plugin for AUBO i5/i7/i10 robots running **old firmware** (legacy ServiceInterface, port 8899). It wraps the pre-built Aubo SDK (`libauborobotcontroller.so`) so the robot can be controlled with standard ROS2 controllers and MoveIt 2.

---

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble installed and sourced

---

## Step 1 — Clone this repository

Clone `aubo_driver` into the `src` folder of your ROS2 workspace:

```bash
cd ~/your_ws/src
git clone https://github.com/harrisonseby/aubo-driver-ros2-wrapper.git
```

---

## Step 2 — Fix the Protobuf Dependency

Protobuf (Protocol Buffers) is a library made by Google for serializing data — the
Aubo SDK uses it internally to encode commands sent over TCP to the robot controller.

The Aubo SDK (`libauborobotcontroller.so`) is a **pre-compiled binary** that was built
on Ubuntu 16.04, which shipped protobuf version 2.6.1 (`libprotobuf.so.9`). It was
compiled against that specific version and cannot be changed without access to Aubo's
source code.

Ubuntu 22.04 ships a much newer protobuf version (`libprotobuf.so.23`). The `.9` and
`.23` are ABI version numbers — they are treated as completely separate libraries by
the system, so the SDK cannot use `.so.23` as a substitute. We need to install `.so.9`
alongside the existing `.so.23`; because the version numbers differ they do not
conflict with each other.

Run the following commands **one at a time**:

**2.1 — Download the old protobuf package from the Ubuntu 16.04 archive:**

```bash
cd ~/Downloads
wget http://archive.ubuntu.com/ubuntu/pool/main/p/protobuf/libprotobuf9v5_2.6.1-1.3_amd64.deb
```

**2.2 — Extract the `.so` file (this does NOT install the whole package system-wide):**

```bash
dpkg-deb -x libprotobuf9v5_2.6.1-1.3_amd64.deb /tmp/proto9
```

**2.3 — Verify the file was extracted:**

```bash
ls /tmp/proto9/usr/lib/x86_64-linux-gnu/
```

You should see `libprotobuf.so.9` and `libprotobuf.so.9.0.1` in the output.

**2.4 — Copy the library to the system library path:**

```bash
sudo cp /tmp/proto9/usr/lib/x86_64-linux-gnu/libprotobuf.so.9* /usr/local/lib/
```

**2.5 — Refresh the system's library cache:**

```bash
sudo ldconfig
```

**2.6 — Confirm it is found:**

```bash
ldconfig -p | grep "libprotobuf.so.9"
```

Expected output:

```
libprotobuf.so.9 (libc6,x86-64) => /usr/local/lib/libprotobuf.so.9
```

If you see that line, the dependency is resolved and you can continue.

---

## Step 3 — Build

From your workspace root:

```bash
cd ~/your_ws
colcon build
source install/setup.bash
```

---

## Troubleshooting

### `libprotobuf.so.9: cannot open shared object file`

The library from Step 2 was not found. Re-run Step 2 from the beginning and make sure Step 2.5 (`sudo ldconfig`) completed without errors.

