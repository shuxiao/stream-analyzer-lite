CXX = g++
BIN = stream-analyzer-core
CXXFLAGS = -std=c++17 -O2 $(shell pkg-config --cflags libavformat libavcodec libavutil libswscale)
LDFLAGS = $(shell pkg-config --libs libavformat libavcodec libavutil libswscale)

$(BIN): main.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(BIN) h264analyzer
