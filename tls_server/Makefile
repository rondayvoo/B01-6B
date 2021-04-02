CXX = g++
CFLAGS = -pthread -lssl -lcrypto
TARGET = tls-alex-server

all: $(TARGET)

$(TARGET): $(TARGET).cpp ; $(CXX) $(TARGET).cpp make_tls_server.cpp tls_server_lib.cpp tls_pthread.cpp tls_common_lib.cpp serial.cpp serialize.cpp $(CFLAGS) -o $(TARGET)

clean: ; $(RM) $(TARGET)
