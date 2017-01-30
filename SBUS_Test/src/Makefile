CXXFLAGS =	-O2 -g -Wall -fmessage-length=0

OBJS =		Uart_Test.o

LIBS =

TARGET =	uart_test

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
