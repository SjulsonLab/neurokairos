CC ?= gcc
CFLAGS ?= -Wall -Wextra -Werror -O2 -std=c11
GPIOD_LIBS := $(shell pkg-config --cflags --libs libgpiod 2>/dev/null || echo -lgpiod)

TARGET = neurokairos-eventlogger
TEST_TARGET = test_eventlogger_core
CORE_SRC = eventlogger_config.c eventlogger_journal.c eventlogger_status.c
TARGET_SRC = eventlogger.c $(CORE_SRC)
TEST_SRC = eventlogger_test.c eventlogger_config.c eventlogger_journal.c eventlogger_status.c

all: $(TARGET)

$(TARGET): $(TARGET_SRC) eventlogger_core.h
	$(CC) $(CFLAGS) -o $(TARGET) $(TARGET_SRC) $(GPIOD_LIBS)

$(TEST_TARGET): $(TEST_SRC) eventlogger_core.h
	$(CC) $(CFLAGS) -o $(TEST_TARGET) $(TEST_SRC)

test: $(TEST_TARGET)
	./$(TEST_TARGET)

clean:
	rm -f $(TARGET) $(TEST_TARGET)

install: $(TARGET)
	sudo cp $(TARGET) /usr/local/bin/
	sudo chmod +x /usr/local/bin/$(TARGET)

.PHONY: all test clean install
