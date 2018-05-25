
CC=gcc
CFLAGS = -g -Wall -lm
EXEC = ./pc-terminal

# C_SOURCE_FILES += \
# $(abspath ./joystick.c) \
# $(abspath ./keyboard.c) \
# $(abspath ../protocol/protocol.c) \

all:
	#$(CC) $(CFLAGS) pc_terminal.c -o $(EXEC)
	$(CC) $(CFLAGS) *.c ../protocol/protocol.c -o $(EXEC)
	#$(CC) $(CFLAGS) *.c joystick.c -o $(EXEC)
	#$(CC) $(CFLAGS) *.c keyboard.c -o $(EXEC)
	#$(CC) $(CFLAGS) $(C_SOURCE_FILES) -o $(EXEC)

run: all
	$(EXEC)
