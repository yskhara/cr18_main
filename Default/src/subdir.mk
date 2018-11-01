################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/cr_can.cpp \
../src/cr_main_auto.cpp \
../src/cr_main_joy.cpp \
../src/cr_main_semiauto.cpp \
../src/tr_can.cpp \
../src/tr_launcher_unitTest.cpp \
../src/tr_main_auto_1v1.cpp \
../src/tr_main_auto_1v2.cpp \
../src/tr_main_auto_2v0.cpp \
../src/tr_main_joy.cpp \
../src/usb_can_node.cpp 

OBJS += \
./src/cr_can.o \
./src/cr_main_auto.o \
./src/cr_main_joy.o \
./src/cr_main_semiauto.o \
./src/tr_can.o \
./src/tr_launcher_unitTest.o \
./src/tr_main_auto_1v1.o \
./src/tr_main_auto_1v2.o \
./src/tr_main_auto_2v0.o \
./src/tr_main_joy.o \
./src/usb_can_node.o 

CPP_DEPS += \
./src/cr_can.d \
./src/cr_main_auto.d \
./src/cr_main_joy.d \
./src/cr_main_semiauto.d \
./src/tr_can.d \
./src/tr_launcher_unitTest.d \
./src/tr_main_auto_1v1.d \
./src/tr_main_auto_1v2.d \
./src/tr_main_auto_2v0.d \
./src/tr_main_joy.d \
./src/usb_can_node.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++1y -I/opt/ros/kinetic/include -I../../../devel/include -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


