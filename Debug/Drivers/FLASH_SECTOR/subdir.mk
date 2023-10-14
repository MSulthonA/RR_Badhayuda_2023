################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/FLASH_SECTOR/FLASH_SECTOR_H7.c 

OBJS += \
./Drivers/FLASH_SECTOR/FLASH_SECTOR_H7.o 

C_DEPS += \
./Drivers/FLASH_SECTOR/FLASH_SECTOR_H7.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/FLASH_SECTOR/%.o Drivers/FLASH_SECTOR/%.su: ../Drivers/FLASH_SECTOR/%.c Drivers/FLASH_SECTOR/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/user/Documents/Teknik Komputer/URDC/RR_Abu_Robocon_STM32/ER_BANDHAYUDHA-2023_core/ER_BANDHAYUDHA-2023_core/Drivers/LCD_character" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/user/Documents/Teknik Komputer/URDC/RR_Abu_Robocon_STM32/ER_BANDHAYUDHA-2023_core/ER_BANDHAYUDHA-2023_core/Drivers/FLASH_SECTOR" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-FLASH_SECTOR

clean-Drivers-2f-FLASH_SECTOR:
	-$(RM) ./Drivers/FLASH_SECTOR/FLASH_SECTOR_H7.d ./Drivers/FLASH_SECTOR/FLASH_SECTOR_H7.o ./Drivers/FLASH_SECTOR/FLASH_SECTOR_H7.su

.PHONY: clean-Drivers-2f-FLASH_SECTOR

