CXX = arm-none-linux-gnueabi-gcc

targets = Lab6_DSPcommshell

all: $(targets)

netapi.o: netapi.c 
	$(CXX) -c netapi.c
	
omapl138_gpiofuncs.o: omapl138_gpiofuncs.c 
	$(CXX) -c omapl138_gpiofuncs.c

Lab6_DSPcommshell.o: Lab6_DSPcommshell.c
	$(CXX) -c Lab6_DSPcommshell.c	 

Lab6_DSPcommshell: netapi.o omapl138_gpiofuncs.o Lab6_DSPcommshell.o
	$(CXX) -o $@ $(CXXFLAGS) $(INCLUDES) $^ 
clean:
	rm -rf *.o $(targets)
  
