# scopehal-pico-bridge

Socket servers for Pico Technology instruments allowing remote access via libscopehal.


## How to use

##### Prerequisites
* Install the Pico SDK (https://picotech.com/downloads)
* Compile [scopehal-apps](https://www.ngscopeclient.org/manual/GettingStarted.html)
* Compile scopehal-pico-bridge (see below for instructions)
---
1. Connect your PicoScope to the computer.
2. Start the scopehal-pico-bridge (ps6000d.exe). It should open a console window and read "Successfully opened instrument", listing some info about the device like model and serial number.
3. Now start [ngscopeclient](https://github.com/ngscopeclient/scopehal-apps). Select Add / Oscilloscope / Connect... from the top menu. Enter or select the following values:
   * Nickname: (choose a display name for your scope.)
   * Driver: **pico**
   * Transport: **twinlan**
   * Path: **localhost:5024:5025**
4. Click Add.
5. If this is your first time using ngscopeclient, continue reading [the tutorial here](https://www.ngscopeclient.org/manual/Tutorials.html).


## How to compile

The process to build from source is basically [the same as for ngscopeclient](https://www.ngscopeclient.org/manual/GettingStarted.html), shown here for **Windows**:

1.
    Download and install MSYS2. You can download it from [msys2.org](https://www.msys2.org/) or [github.com/msys2/msys2-installer/releases](https://github.com/msys2/msys2-installer/releases).
    The following steps can be done in any MSYS-provided shell (it will start up with a UCRT64 shell after installation).
2.
    Install git and the toolchain:
	```
    pacman -S git wget mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-toolchain
    ```
3.
    Install general dependencies:
    ```
    pacman -S mingw-w64-ucrt-x86_64-libsigc++ mingw-w64-ucrt-x86_64-yaml-cpp mingw-w64-ucrt-x86_64-glfw mingw-w64-ucrt-x86_64-catch mingw-w64-ucrt-x86_64-hidapi mingw-w64-ucrt-x86_64-libpng
    ```
4.
    Install Vulkan dependencies:
    ```
    pacman -S mingw-w64-ucrt-x86_64-vulkan-headers mingw-w64-ucrt-x86_64-vulkan-loader mingw-w64-ucrt-x86_64-shaderc mingw-w64-ucrt-x86_64-glslang mingw-w64-ucrt-x86_64-spirv-tools
    ```
5.
    Install FFTS:
    ```
    pacman -S mingw-w64-ucrt-x86_64-ffts
    ```
6.
    Check out the code
    ```
    cd ~ 
    git clone --recursive https://github.com/ngscopeclient/scopehal-pico-bridge
    ```

    **All following steps are to be done in a UCRT64 shell.**

7.
    Build manually:
    ```
    cd scopehal-pico-bridge
    mkdir build
    cd build
    cmake ..
    ninja -j4
    ```
	If the build fails with a lot of missing files, try adding your MSYS2 bin folder to your $PATH variable, i.e. "C:\msys64\ucrt64\bin" if you installed it to "C:\msys64".
8.
    To run scopehal-pico-bridge:
    The binary can be found in the build directory, such as $HOME/scopehal-pico-bridge/build/src/ps6000d.
