VAMP_SDK_PATH=/home/chrisbau/builds/vamp-plugin-sdk-v2.2.1
LIBS=-lvamp-hostsdk -lportaudio -ldl -lsndfile
FLAGS=-g -Wall

vamp-live-host: vamplivehost.cpp
	g++ vamplivehost.cpp -o vamp-live-host $(FLAGS) -I$(VAMP_SDK_PATH) $(LIBS)

.PHONY: clean

clean:
	rm -f vamp-live-host
