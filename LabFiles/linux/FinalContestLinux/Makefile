CXX = arm-none-linux-gnueabi-gcc

targets = Robot2CommShell

all: $(targets)

netapi.o: netapi.c 
	$(CXX) -c netapi.c
	
omapl138_gpiofuncs.o: omapl138_gpiofuncs.c 
	$(CXX) -c omapl138_gpiofuncs.c

Robot2CommShell.o: Robot2CommShell.c
	$(CXX) -c Robot2CommShell.c	 

Robot2CommShell: netapi.o omapl138_gpiofuncs.o Robot2CommShell.o
	$(CXX) -o $@ $(CXXFLAGS) $(INCLUDES) $^ 
clean:
	rm -rf *.o $(targets)
  
