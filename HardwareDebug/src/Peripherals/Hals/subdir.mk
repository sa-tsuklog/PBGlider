################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
..\src/Peripherals/Hals/HalFlash.cpp \
..\src/Peripherals/Hals/HalI2C.cpp \
..\src/Peripherals/Hals/HalRangeFinderMtu.cpp \
..\src/Peripherals/Hals/HalSci0.cpp \
..\src/Peripherals/Hals/HalSci5.cpp \
..\src/Peripherals/Hals/HalServoMtus.cpp 

OBJS += \
./src/Peripherals/Hals/HalFlash.obj \
./src/Peripherals/Hals/HalI2C.obj \
./src/Peripherals/Hals/HalRangeFinderMtu.obj \
./src/Peripherals/Hals/HalSci0.obj \
./src/Peripherals/Hals/HalSci5.obj \
./src/Peripherals/Hals/HalServoMtus.obj 

CPP_DEPS += \
./src/Peripherals/Hals/HalFlash.d \
./src/Peripherals/Hals/HalI2C.d \
./src/Peripherals/Hals/HalRangeFinderMtu.d \
./src/Peripherals/Hals/HalSci0.d \
./src/Peripherals/Hals/HalSci5.d \
./src/Peripherals/Hals/HalServoMtus.d 


# Each subdirectory must supply rules for building sources it contributes
src/Peripherals/Hals/%.obj src/Peripherals/Hals/%.d: ../src/Peripherals/Hals/%.cpp
	@echo 'Scanning and building file: $<'
	@echo 'Invoking: Scanner and Compiler'
	scandep1 -MM -MP -MF "$(@:%.obj=%.d)" -MT "$(@:%.obj=%.d)"    -I"E:/MyDocument/Dropbox/EmbededProjects/62N_PBGlider" -I"C:\PROGRA~2\Renesas\Hew\Tools\Renesas\RX\1_2_1\include" -D__RX   -D__RX600=1  -D__LIT=1 -D__FPU=1 -D__RON=1 -D__UCHAR=1 -D__DBL4=1 -D__UBIT=1 -D__BITRIGHT=1 -D__DOFF=1 -D__cplusplus=1   -D__RENESAS__=1 -D__RENESAS_VERSION__=0x010201 -D__RX=1 -D__STDC__=1   -E -quiet -I. -CC "$<"
	ccrx -output=obj="$(@:%.d=%.obj)"  -include="C:\PROGRA~2\Renesas\Hew\Tools\Renesas\RX\1_2_1\include"  -debug -nologo -change_message=warning -cpu=rx600  -define=__RX   -lang=cpp "$<"
	@echo 'Finished scanning and building: $<'
	@echo.

