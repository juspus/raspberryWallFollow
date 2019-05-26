all: Robot PowerControl

HDRS = I2Cdev.hpp VL53L0X_defines.hpp VL53L0X.hpp Global.h
OBJ = I2Cdev.o VL53L0X.o MPU6050.o pwm.o bluetooth.o encoder.o wallAproximation.o 
EXEC = Robot.o

$(OBJ) $(EXEC) : $(HDRS)

Robot: $(OBJ) $(EXEC)
	$(CXX) -o $@ $^ -pthread -lwiringPi -lbluetooth

PowerControl:
	$(CXX) -o powercontrol PowerControl.cpp -lwiringPi

clean:
	rm -f $(OBJ) $(EXEC) Robot powercontrol