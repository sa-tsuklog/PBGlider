################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
..\src/Peripherals/Bmp085.cpp \
..\src/Peripherals/GpsCrReceivedHandler.cpp \
..\src/Peripherals/Inu.cpp \
..\src/Peripherals/KalmanFilter.cpp \
..\src/Peripherals/KalmanFilter_2x2.cpp \
..\src/Peripherals/Mpu9150.cpp \
..\src/Peripherals/OrientationFilter.cpp \
..\src/Peripherals/Quaternion.cpp \
..\src/Peripherals/RangeFinder.cpp \
..\src/Peripherals/Servo.cpp \
..\src/Peripherals/StdSci.cpp \
..\src/Peripherals/Storage.cpp 

OBJS += \
./src/Peripherals/Bmp085.obj \
./src/Peripherals/GpsCrReceivedHandler.obj \
./src/Peripherals/Inu.obj \
./src/Peripherals/KalmanFilter.obj \
./src/Peripherals/KalmanFilter_2x2.obj \
./src/Peripherals/Mpu9150.obj \
./src/Peripherals/OrientationFilter.obj \
./src/Peripherals/Quaternion.obj \
./src/Peripherals/RangeFinder.obj \
./src/Peripherals/Servo.obj \
./src/Peripherals/StdSci.obj \
./src/Peripherals/Storage.obj 

CPP_DEPS += \
./src/Peripherals/Bmp085.d \
./src/Peripherals/GpsCrReceivedHandler.d \
./src/Peripherals/Inu.d \
./src/Peripherals/KalmanFilter.d \
./src/Peripherals/KalmanFilter_2x2.d \
./src/Peripherals/Mpu9150.d \
./src/Peripherals/OrientationFilter.d \
./src/Peripherals/Quaternion.d \
./src/Peripherals/RangeFinder.d \
./src/Peripherals/Servo.d \
./src/Peripherals/StdSci.d \
./src/Peripherals/Storage.d 


# Each subdirectory must supply rules for building sources it contributes
src/Peripherals/%.obj src/Peripherals/%.d: ../src/Peripherals/%.cpp
	@echo 'Scanning and building file: $<'
	@echo 'Invoking: Scanner and Compiler'
	scandep1 -MM -MP -MF "$(@:%.obj=%.d)" -MT "$(@:%.obj=%.d)"    -I"E:/MyDocument/Dropbox/EmbededProjects/62N_PBGlider" -I"C:\PROGRA~2\Renesas\Hew\Tools\Renesas\RX\1_2_1\include" -D__RX   -D__RX600=1  -D__LIT=1 -D__FPU=1 -D__RON=1 -D__UCHAR=1 -D__DBL4=1 -D__UBIT=1 -D__BITRIGHT=1 -D__DOFF=1 -D__cplusplus=1   -D__RENESAS__=1 -D__RENESAS_VERSION__=0x010201 -D__RX=1 -D__STDC__=1   -E -quiet -I. -CC "$<"
	ccrx -output=obj="$(@:%.d=%.obj)"  -include="C:\PROGRA~2\Renesas\Hew\Tools\Renesas\RX\1_2_1\include"  -debug -nologo -change_message=warning -cpu=rx600  -define=__RX   -lang=cpp "$<"
	@echo 'Finished scanning and building: $<'
	@echo.

