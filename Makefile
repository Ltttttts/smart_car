# Smart Car — Makefile
# Target: Orange Pi 5B (aarch64), Ubuntu 22.04/24.04
# Author: Ltttttts

CC       := gcc
CFLAGS   := -Wall -Wextra -Wconversion -Wshadow \
            -std=c11 -O2 -g
LDFLAGS  := -lm -lpthread

SRCDIR   := src
BINDIR   := build
TARGET   := $(BINDIR)/smart_car

SRCS     := $(wildcard $(SRCDIR)/*.c)
OBJS     := $(patsubst $(SRCDIR)/%.c, $(BINDIR)/%.o, $(SRCS))
DEPS     := $(OBJS:.o=.d)

INC      := -I$(SRCDIR)

.PHONY: all clean run help

all: $(TARGET)

# Create build directory
$(BINDIR):
	mkdir -p $(BINDIR)

# Link
$(TARGET): $(OBJS) | $(BINDIR)
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(LDFLAGS)
	@echo "  LINK  $@"

# Compile with auto-dependency generation
$(BINDIR)/%.o: $(SRCDIR)/%.c | $(BINDIR)
	$(CC) $(CFLAGS) $(INC) -MMD -MP -c $< -o $@
	@echo "  CC    $<"

# Include auto-generated dependency files
-include $(DEPS)

# Run with default device
run: $(TARGET)
	@echo "Running demo..."
	sudo $(TARGET) /dev/ttyUSB0

# Clean build artifacts
clean:
	rm -rf $(BINDIR)

# Create clean source tarball (excludes build/, node_modules/)
dist:
	tar czf smart_car_src.tar.gz \
	    --exclude='build' \
	    --exclude='node_modules' \
	    --exclude='.git' \
	    --exclude='*.tar.gz' \
	    src/ Makefile PROPOSAL.md "reference data" .gitignore
	@echo "  DIST  smart_car_src.tar.gz"

help:
	@echo "Usage:"
	@echo "  make          Build the project"
	@echo "  make run      Build and run with /dev/ttyUSB0"
	@echo "  make clean    Remove build artifacts"
	@echo "  make dist     Create clean source tarball"
	@echo ""
	@echo "  DEVICE=/dev/ttyS0 make run   (custom device)"
