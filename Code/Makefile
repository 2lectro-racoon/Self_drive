CC := g++

OUTPUT	= program
SRCS	= ./src/main.cpp ./src/base64.cpp

CFLAGS	= -I./lib -I/usr/local/include/opencv4 -Wall -std=c++17
LDFLAGS	= -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -pthread -lopencv_objdetect -lpigpio -lrt -lcpr

$(OUTPUT): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f $(OUTPUT)

# CC := g++

# OUTPUT	= program
# SRCS	= ./src/main.cpp \ ./src/base64.cpp

# CFLAGS	= -I./lib -I/usr/local/include/opencv4 -Wall -std=c++11
# LDFLAGS	= -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -pthread -lopencv_objdetect -lpigpio -lrt -lcpr -ljson

# $(OUTPUT) : $(SRCS)
# 	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDFLAGS)

# clean:
# 	@rm -f $(OUTPUT)