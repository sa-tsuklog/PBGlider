################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
..\src/62N_PBGlider.cpp \
..\src/ControlLogic.cpp \
..\src/ControlLoop.cpp \
..\src/GeneralConfig.cpp \
..\src/Util.cpp 

C_SRCS += \
..\src/dbsct.c \
..\src/interrupt_handlers.c \
..\src/reset_program.c \
..\src/sbrk.c \
..\src/vector_table.c 

OBJS += \
./src/62N_PBGlider.obj \
./src/ControlLogic.obj \
./src/ControlLoop.obj \
./src/GeneralConfig.obj \
./src/Util.obj \
./src/dbsct.obj \
./src/interrupt_handlers.obj \
./src/reset_program.obj \
./src/sbrk.obj \
./src/vector_table.obj 

C_DEPS += \
./src/dbsct.d \
./src/interrupt_handlers.d \
./src/reset_program.d \
./src/sbrk.d \
./src/vector_table.d 

CPP_DEPS += \
./src/62N_PBGlider.d \
./src/ControlLogic.d \
./src/ControlLoop.d \
./src/GeneralConfig.d \
./src/Util.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.obj src/%.d: ../src/%.cpp
	@echo 'Scanning and building file: $<'
	@echo 'Invoking: Scanner and Compiler'
	scandep1 -MM -MP -MF "$(@:%.obj=%.d)" -MT "$(@:%.obj=%.d)"    -I"E:/MyDocument/Dropbox/EmbededProjects/62N_PBGlider" -I"C:\PROGRA~2\Renesas\Hew\Tools\Renesas\RX\1_2_1\include" -D__RX   -D__RX600=1  -D__LIT=1 -D__FPU=1 -D__RON=1 -D__UCHAR=1 -D__DBL4=1 -D__UBIT=1 -D__BITRIGHT=1 -D__DOFF=1 -D__cplusplus=1   -D__RENESAS__=1 -D__RENESAS_VERSION__=0x010201 -D__RX=1 -D__STDC__=1   -E -quiet -I. -CC "$<"
	ccrx -output=obj="$(@:%.d=%.obj)"  -include="C:\PROGRA~2\Renesas\Hew\Tools\Renesas\RX\1_2_1\include"  -debug -nologo -change_message=warning -cpu=rx600  -define=__RX   -lang=cpp "$<"
	@echo 'Finished scanning and building: $<'
	@echo.

src/%.obj src/%.d: ../src/%.c
	@echo 'Scanning and building file: $<'
	@echo 'Invoking: Scanner and Compiler'
	scandep1 -MM -MP -MF "$(@:%.obj=%.d)" -MT "$(@:%.obj=%.d)"    -I"E:/MyDocument/Dropbox/EmbededProjects/62N_PBGlider" -I"C:\PROGRA~2\Renesas\Hew\Tools\Renesas\RX\1_2_1\include" -D__RX   -D__RX600=1  -D__LIT=1 -D__FPU=1 -D__RON=1 -D__UCHAR=1 -D__DBL4=1 -D__UBIT=1 -D__BITRIGHT=1 -D__DOFF=1 -D__STDC_VERSION__=199409L   -D__RENESAS__=1 -D__RENESAS_VERSION__=0x010201 -D__RX=1 -D__STDC__=1   -E -quiet -I. -C "$<"
	ccrx -output=obj="$(@:%.d=%.obj)"  -include="C:\PROGRA~2\Renesas\Hew\Tools\Renesas\RX\1_2_1\include"  -debug -nologo -change_message=warning -cpu=rx600  -define=__RX   -lang=c "$<"
	@echo 'Finished scanning and building: $<'
	@echo.

