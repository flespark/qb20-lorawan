# QB20 LoRaWAN

SWL2001 porting for Lierda QB20 EVK, basic lorawan feature support, developing on STM32CubeMX and VSCode.

# Setup

1. Install toolchain, recommand using `scoop` under Windows:

   ```shell
   scoop install vscode cmake ninja gcc-arm-none-eabi clangd ccache openocd
   ```

   `ccache` and `openocd` is optional，extra `arm-none-eabi-newlib` maybe required under Linux.
2. Flash and debug MCU by Jlink or STLink and Openocd，install corresponding software.
3. Open repo as folder in VSCode, install recommanded extension, cmake build, flash and debug by bottom click:

   .![img](./doc/image/vscode-action-button.png)

# Note

Maybe need full erase MCU flash before flashing because NVM context will cause join fail when used gateway not compatible newest LoRaWAN specification.
