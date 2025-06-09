# Compiler and flags
CC = g++
CFLAGS = -Wall -O3 -g -ffast-math

# Source files
SRCS = neon_test.cpp timeit.cpp
OBJS = $(SRCS:.c=.o)

# Output binary name
TARGET = neon_test

# Default target
all: $(TARGET)

# Link the object files into the binary
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

# Compile .c to .o
%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -f $(TARGET)
