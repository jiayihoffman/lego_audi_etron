# SimpleBLE
ROS2 control hardware interfaces are C++ only; Python isn't supported. The C++ implementation of the Bluetooth lib is SimpleBLE.

## Options
### Option 1: Check if system package exists (quickest)

```
# Check if SimpleBLE is available as a package
sudo apt update
sudo apt search simpleble

# If available, install it
sudo apt install libsimpleble-dev
```

### Option 2: Build from source (recommended)

Step 1: Install dependencies
```
# Required dependencies
sudo apt-get update
sudo apt-get install -y \
    cmake \
    build-essential \
    libdbus-1-dev \
    pkg-config \
    git
```

Step 2: Clone and build SimpleBLE
```
# Clone the repository
cd ~/  # or your preferred location
git clone https://github.com/OpenBluetoothToolbox/SimpleBLE.git
cd SimpleBLE/simpleble

# Create build directory
mkdir build
cd build

# Configure with CMake
# Install to /usr/local (default) or specify custom prefix
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build (use -j$(nproc) for parallel build)
cmake --build . --parallel

# Install to system (requires sudo)
sudo cmake --install .
```

Step 3: Update library cache
```
# Update library cache so the system can find SimpleBLE
sudo ldconfig
```

Step 4: Verify installation
```
# Check if pkg-config can find it
pkg-config --modversion simpleble
```