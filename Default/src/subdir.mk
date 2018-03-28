################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/cr_main_joy.cpp \
../src/tr_can.cpp \
../src/tr_launcher_unitTest.cpp \
../src/tr_main_auto_1v1.cpp \
../src/tr_main_auto_1v2.cpp \
../src/tr_main_joy.cpp \
../src/usb_can_node.cpp 

OBJS += \
./src/cr_main_joy.o \
./src/tr_can.o \
./src/tr_launcher_unitTest.o \
./src/tr_main_auto_1v1.o \
./src/tr_main_auto_1v2.o \
./src/tr_main_joy.o \
./src/usb_can_node.o 

CPP_DEPS += \
./src/cr_main_joy.d \
./src/tr_can.d \
./src/tr_launcher_unitTest.d \
./src/tr_main_auto_1v1.d \
./src/tr_main_auto_1v2.d \
./src/tr_main_joy.d \
./src/usb_can_node.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++1y -I/opt/ros/kinetic/include -I../../../devel/include -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


