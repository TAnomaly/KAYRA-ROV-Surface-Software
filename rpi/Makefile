# ──────────────────────────────────────────────────────────────
#  KAYRA ROV — Onboard Controller (Raspberry Pi 5)
# ──────────────────────────────────────────────────────────────

CC       = gcc
CFLAGS   = -Wall -Wextra -Wpedantic -std=c11 -D_DEFAULT_SOURCE -O2
CFLAGS  += -Iinclude
LDFLAGS  = -lm

SRC_DIR  = src
OBJ_DIR  = build
TARGET   = kayra-rov

SRCS     = $(wildcard $(SRC_DIR)/*.c)
OBJS     = $(SRCS:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

.PHONY: all clean install

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
	@echo ""
	@echo "  Build complete: ./$(TARGET)"
	@echo "  Run:   ./$(TARGET)"
	@echo "  Help:  ./$(TARGET) --help"
	@echo ""

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

clean:
	rm -rf $(OBJ_DIR) $(TARGET)

install:
	@echo "Installing systemd service..."
	sudo cp kayra-rov.service /etc/systemd/system/
	sudo systemctl daemon-reload
	sudo systemctl enable kayra-rov
	@echo "Service installed. Start with: sudo systemctl start kayra-rov"
