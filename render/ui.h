#pragma once

#include <raylib.h>

#include "core/common.h"

// A small immediate-mode UI for the viewer: panels, buttons, toggle switches and a
// segmented control, drawn in JetBrains Mono (embedded at build time; Raylib's own font if
// the build had none). Each widget is drawn and handles its click in one call.
//
// Input routing: every panel registers the area it covers, and is_mouse_over_ui answers
// from the previous frame's areas, so the 3D view can ignore the mouse over the UI before
// this frame's UI is drawn.

enum Ui_Font_Kind {
    UI_FONT_SMALL, // captions, hints, shortcut keys
    UI_FONT_BODY, // labels and values
    UI_FONT_BOLD, // headings, the wordmark, button text
    UI_FONT_COUNT,
};

struct Ui_Theme {
    Color bar; // top and bottom bars
    Color panel;
    Color border;
    Color control; // button and track backgrounds
    Color control_hover;
    Color control_pressed;
    Color text;
    Color text_dim;
    Color text_disabled;
    Color green;
    Color amber;
    Color red;
    Color blue;
};

#define UI_MAX_REGIONS 16
#define UI_MAX_ANIMATIONS 16

// A toggle's knob position, eased towards its value; keyed by the value's address.
struct Ui_Animation {
    const void *key;
    f32 position; // 0 off .. 1 on
};

struct Ui {
    Font fonts[UI_FONT_COUNT];
    f32 font_sizes[UI_FONT_COUNT];
    bool embedded_fonts; // false: Raylib's built-in font
    Ui_Theme theme;
    Vector2 mouse;
    bool mouse_pressed; // the left button went down this frame
    bool mouse_down;
    f32 frame_seconds;
    Rectangle regions[UI_MAX_REGIONS]; // covered this frame
    u32 region_count;
    Rectangle previous_regions[UI_MAX_REGIONS]; // covered last frame, for input routing
    u32 previous_region_count;
    Ui_Animation animations[UI_MAX_ANIMATIONS];
    u32 animation_count;
};

// Needs the window (it uploads font textures).
void create_ui(Ui *ui);
void destroy_ui(Ui *ui);

// At the start of each frame, before any input is handled.
void begin_ui(Ui *ui);
bool is_mouse_over_ui(const Ui *ui);

// A dark panel, registered as covering its area.
void draw_ui_panel(Ui *ui, Rectangle area, Color color);

void draw_ui_text(const Ui *ui, Ui_Font_Kind kind, const char *text, f32 x, f32 y, Color color);
f32 measure_ui_text(const Ui *ui, Ui_Font_Kind kind, const char *text);
f32 get_ui_font_size(const Ui *ui, Ui_Font_Kind kind);

// A small caps heading with a rule under it. Advances *y past it.
void draw_ui_section(const Ui *ui, const char *title, f32 x, f32 *y, f32 width);

// A button with an optional shortcut shown dimly at its right. True when clicked.
bool draw_ui_button(Ui *ui, Rectangle area, const char *label, const char *shortcut, bool enabled);

// A row with a label (and optional shortcut) on the left and a switch on the right. Flips
// *value and returns true when clicked.
bool draw_ui_toggle(Ui *ui, Rectangle row, const char *label, const char *shortcut, bool *value,
                    bool enabled);

// Side-by-side choices; the selected one is filled with its accent. Returns the index
// clicked this frame, or -1.
i32 draw_ui_segmented(Ui *ui, Rectangle area, const char *const *labels, const Color *accents,
                      u32 count, u32 selected);

void draw_ui_dot(f32 x, f32 y, Color color);
