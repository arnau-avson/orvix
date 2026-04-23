# lens - build targets
#
# Local Linux:        make
# Android NDK cross:  make android NDK=/path/to/ndk API=30 ARCH=x86_64 HOST=linux-x86_64

CC      ?= clang
CFLAGS  ?= -O3 -Wall -Wextra -std=c11
TARGET   = lens
SRC      = lens.c

NDK     ?=
API     ?= 30
ARCH    ?= x86_64        # x86_64 | aarch64 | armv7a | i686
HOST    ?= windows-x86_64

.PHONY: all clean android push run

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $<

android: $(SRC)
ifndef NDK
	$(error Set NDK=/path/to/android-ndk-rXX)
endif
	$(NDK)/toolchains/llvm/prebuilt/$(HOST)/bin/$(ARCH)-linux-android$(API)-clang \
		$(CFLAGS) -o $(TARGET) $<

push: $(TARGET)
	adb push $(TARGET) /data/local/tmp/$(TARGET)
	adb shell chmod 755 /data/local/tmp/$(TARGET)

run: push
	adb shell /data/local/tmp/$(TARGET) shot http://127.0.0.1:8080/upload

clean:
	rm -f $(TARGET) $(TARGET).tmp
