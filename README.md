# STM32Cube project for Portenta H7

This repo contains an STM32Cube project tailored for Arduino Portenta H7 board.

Beside the pinmuxing and correct clock configuration, we added some code in main.c to properly initialize the board:

* Initialize clock main.c#L213
* Take USB ULPI chip out of reset main.c#L215
* Initialize PMIC rails main.c#L270-L336

## Known limitations
* Creating a project will generate a non compiling makefile; use the following patch to fix it
  ```patch
  246a247,249
  > ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
  > ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
  > ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
  247a251,254
  > ../../Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.c \
  > ../../Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.c \
  > ../../Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.c \
  > ../../Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.c \
  381,382c388,389
  < -LMiddlewares/ST/STM32_Audio/Addons/PDM/Lib \
  < -LMiddlewares/ST/STM32_Audio/Addons/PDM/Lib
  ---
  > -L../../Middlewares/ST/STM32_Audio/Addons/PDM/Lib \
  > -L../../Middlewares/ST/STM32_Audio/Addons/PDM/Lib
  ```
* I2S2 and SPI2 cannot be configured together; the pinux is however already prepared (if you need to use I2S2)
* SDMMC{1/2} are not initilized at startup since they fail if the SDCARD is not inserted. Call `MX_SDMMC1_MMC_Init` or `MX_SDMMC2_SD_Init` manually if you know what you are doing
