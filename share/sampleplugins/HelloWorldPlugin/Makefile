
CXXFLAGS += `pkg-config --cflags choreonoid`
PLUGIN = libCnoidHelloWorldPlugin.so

$(PLUGIN): HelloWorldPlugin.o
	g++ -shared `pkg-config --libs choreonoid` -o $(PLUGIN) HelloWorldPlugin.o 

install: $(PLUGIN)
	install -s $(PLUGIN) `pkg-config --variable=plugindir choreonoid`
clean:
	rm -f *.o *.so
