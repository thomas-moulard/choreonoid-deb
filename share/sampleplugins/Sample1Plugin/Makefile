
CXXFLAGS += `pkg-config --cflags choreonoid-body-plugin`
PLUGIN = libCnoidSample1Plugin.so
SRC = Sample1Plugin.o

$(PLUGIN): $(SRC)
	g++ -shared `pkg-config --libs choreonoid-body-plugin` -o $(PLUGIN) $(SRC)

install: $(PLUGIN)
	install -s $(PLUGIN) `pkg-config --variable=plugindir choreonoid`
clean:
	rm -f *.o *.so
