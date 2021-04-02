CXX = g++
CFLAGS = -pthread -lssl -lcrypto
TARGET = tls-alex-client

all: $(TARGET)

$(TARGET): $(TARGET).cpp ; $(CXX) $(TARGET).cpp make_tls_client.cpp tls_client_lib.cpp tls_pthread.cpp tls_common_lib.cpp $(CFLAGS) -o $(TARGET)

clean: ; $(RM) $(TARGET)
