# ──────────────────────────────────────────────────────────────
#  Surface Control — ROV Ground Station
# ──────────────────────────────────────────────────────────────

CC        = gcc
CXX       = g++

CFLAGS    = -Wall -Wextra -Wpedantic -std=c11 -D_DEFAULT_SOURCE -O2
CFLAGS   += -Iinclude
CFLAGS   += $(shell pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0)

CXXFLAGS  = -Wall -Wextra -std=c++17 -O2
CXXFLAGS += -Iinclude

# Dear ImGui
IMGUI_DIR  = third_party/imgui
CXXFLAGS  += -I$(IMGUI_DIR) -I$(IMGUI_DIR)/backends

# SDL2 include path (imgui backends use <SDL.h> not <SDL2/SDL.h>)
SDL2_CFLAGS = $(shell sdl2-config --cflags)

# Quiet flags for third-party code
IMGUI_CXX  = -std=c++17 -O2 $(SDL2_CFLAGS) -I$(IMGUI_DIR) -I$(IMGUI_DIR)/backends -Iinclude

LDFLAGS    = -lSDL2 -lGL -ldl -lm -lpthread
LDFLAGS   += $(shell pkg-config --libs gstreamer-1.0 gstreamer-app-1.0)

# ── sources ───────────────────────────────────────────────────

SRC_DIR  = src
OBJ_DIR  = build
TARGET   = surface-control

C_SRCS   = $(wildcard $(SRC_DIR)/*.c)
CXX_SRCS = $(wildcard $(SRC_DIR)/*.cpp)

C_OBJS   = $(C_SRCS:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
CXX_OBJS = $(CXX_SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

IMGUI_OBJS = $(OBJ_DIR)/imgui.o            \
             $(OBJ_DIR)/imgui_draw.o        \
             $(OBJ_DIR)/imgui_tables.o      \
             $(OBJ_DIR)/imgui_widgets.o     \
             $(OBJ_DIR)/imgui_impl_sdl2.o   \
             $(OBJ_DIR)/imgui_impl_opengl3.o

OBJS = $(C_OBJS) $(CXX_OBJS) $(IMGUI_OBJS)

# ── targets ───────────────────────────────────────────────────

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
	@echo ""
	@echo "  Build complete: ./$(TARGET)"
	@echo "  CLI mode:       ./$(TARGET)"
	@echo "  GUI mode:       ./$(TARGET) --gui"
	@echo ""

# ── our C sources ─────────────────────────────────────────────

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c -o $@ $<

# ── our C++ sources ───────────────────────────────────────────

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# ── Dear ImGui core ──────────────────────────────────────────

$(OBJ_DIR)/imgui.o: $(IMGUI_DIR)/imgui.cpp | $(OBJ_DIR)
	$(CXX) $(IMGUI_CXX) -c -o $@ $<

$(OBJ_DIR)/imgui_draw.o: $(IMGUI_DIR)/imgui_draw.cpp | $(OBJ_DIR)
	$(CXX) $(IMGUI_CXX) -c -o $@ $<

$(OBJ_DIR)/imgui_tables.o: $(IMGUI_DIR)/imgui_tables.cpp | $(OBJ_DIR)
	$(CXX) $(IMGUI_CXX) -c -o $@ $<

$(OBJ_DIR)/imgui_widgets.o: $(IMGUI_DIR)/imgui_widgets.cpp | $(OBJ_DIR)
	$(CXX) $(IMGUI_CXX) -c -o $@ $<

# ── Dear ImGui backends ──────────────────────────────────────

$(OBJ_DIR)/imgui_impl_sdl2.o: $(IMGUI_DIR)/backends/imgui_impl_sdl2.cpp | $(OBJ_DIR)
	$(CXX) $(IMGUI_CXX) -c -o $@ $<

$(OBJ_DIR)/imgui_impl_opengl3.o: $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp | $(OBJ_DIR)
	$(CXX) $(IMGUI_CXX) -c -o $@ $<

# ── build dir ─────────────────────────────────────────────────

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

clean:
	rm -rf $(OBJ_DIR) $(TARGET)
