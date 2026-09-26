#include "render/ui.h"

#include <string.h>

// JetBrains Mono, embedded by CMake from REGOLITH_FONT_REGULAR and REGOLITH_FONT_BOLD
// (generated ui_fonts.cpp). Empty when the build had no font.
extern const u8 ui_font_regular[];
extern const u64 ui_font_regular_size;
extern const u8 ui_font_bold[];
extern const u64 ui_font_bold_size;

#define UI_RADIUS 4.0f // px, corner radius
#define UI_KNOB_SPEED 12.0f // toggle knob travel per second, in widths

static Color make_color(u32 rgb, u8 alpha = 255)
{
    return Color{(u8)(rgb >> 16), (u8)(rgb >> 8), (u8)rgb, alpha};
}

// Printable ASCII plus the few symbols the viewer writes.
static i32 font_codepoints[128];
static i32 font_codepoint_count;

static Font load_embedded_font(const u8 *data, u64 size, f32 pixels)
{
    Font font = LoadFontFromMemory(".ttf", data, (i32)size, (i32)pixels, font_codepoints,
                                   font_codepoint_count);
    SetTextureFilter(font.texture, TEXTURE_FILTER_BILINEAR);
    return font;
}

void create_ui(Ui *ui)
{
    *ui = {};
    ui->theme = Ui_Theme{
        .bar = make_color(0x0f1115, 240),
        .panel = make_color(0x15171c, 235),
        .border = make_color(0x2a2e36),
        .control = make_color(0x22262e),
        .control_hover = make_color(0x2c313b),
        .control_pressed = make_color(0x363c48),
        .text = make_color(0xe6e8eb),
        .text_dim = make_color(0x8a919c),
        .text_disabled = make_color(0x4b515b),
        .green = make_color(0x3fb950),
        .amber = make_color(0xd29922),
        .red = make_color(0xf85149),
        .blue = make_color(0x58a6ff),
    };
    ui->font_sizes[UI_FONT_SMALL] = 13.0f;
    ui->font_sizes[UI_FONT_BODY] = 15.0f;
    ui->font_sizes[UI_FONT_BOLD] = 15.0f;

    font_codepoint_count = 0;
    for (i32 c = 32; c < 127; c++) {
        font_codepoints[font_codepoint_count++] = c;
    }
    const i32 symbols[] = {0x00b0, 0x00b7, 0x00d7, 0x2013, 0x2014, 0x2192, 0x25cf};
    for (u32 i = 0; i < ARRAY_COUNT(symbols); i++) {
        font_codepoints[font_codepoint_count++] = symbols[i];
    }

    ui->embedded_fonts = ui_font_regular_size > 0 && ui_font_bold_size > 0;
    if (ui->embedded_fonts) {
        ui->fonts[UI_FONT_SMALL] = load_embedded_font(ui_font_regular, ui_font_regular_size,
                                                      ui->font_sizes[UI_FONT_SMALL]);
        ui->fonts[UI_FONT_BODY] =
            load_embedded_font(ui_font_regular, ui_font_regular_size, ui->font_sizes[UI_FONT_BODY]);
        ui->fonts[UI_FONT_BOLD] =
            load_embedded_font(ui_font_bold, ui_font_bold_size, ui->font_sizes[UI_FONT_BOLD]);
    } else {
        log_info("viewer: built without an embedded font; using Raylib's");
        for (u32 i = 0; i < UI_FONT_COUNT; i++) {
            ui->fonts[i] = GetFontDefault();
        }
    }
}

void destroy_ui(Ui *ui)
{
    if (ui->embedded_fonts) {
        for (u32 i = 0; i < UI_FONT_COUNT; i++) {
            UnloadFont(ui->fonts[i]);
        }
    }
    *ui = {};
}

void begin_ui(Ui *ui)
{
    memcpy(ui->previous_regions, ui->regions, sizeof(ui->regions));
    ui->previous_region_count = ui->region_count;
    ui->region_count = 0;
    ui->mouse = GetMousePosition();
    ui->mouse_pressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
    ui->mouse_down = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
    ui->frame_seconds = GetFrameTime();
}

bool is_mouse_over_ui(const Ui *ui)
{
    Vector2 mouse = GetMousePosition();
    for (u32 i = 0; i < ui->previous_region_count; i++) {
        if (CheckCollisionPointRec(mouse, ui->previous_regions[i])) {
            return true;
        }
    }
    return false;
}

void draw_ui_panel(Ui *ui, Rectangle area, Color color)
{
    DrawRectangleRec(area, color);
    if (ui->region_count < UI_MAX_REGIONS) {
        ui->regions[ui->region_count++] = area;
    }
}

f32 get_ui_font_size(const Ui *ui, Ui_Font_Kind kind)
{
    // Raylib's font is a 10 px bitmap: only whole multiples of that stay sharp.
    if (!ui->embedded_fonts) {
        return kind == UI_FONT_SMALL ? 10.0f : 20.0f;
    }
    return ui->font_sizes[kind];
}

// Mono text needs no extra spacing; Raylib's font does.
static f32 get_spacing(const Ui *ui) { return ui->embedded_fonts ? 0.0f : 1.0f; }

void draw_ui_text(const Ui *ui, Ui_Font_Kind kind, const char *text, f32 x, f32 y, Color color)
{
    // Whole pixels keep the glyphs sharp.
    Vector2 position = {(f32)(i32)(x + 0.5f), (f32)(i32)(y + 0.5f)};
    DrawTextEx(ui->fonts[kind], text, position, get_ui_font_size(ui, kind), get_spacing(ui), color);
}

f32 measure_ui_text(const Ui *ui, Ui_Font_Kind kind, const char *text)
{
    return MeasureTextEx(ui->fonts[kind], text, get_ui_font_size(ui, kind), get_spacing(ui)).x;
}

// Text centred vertically in a box of the given height starting at top.
static f32 get_text_top(const Ui *ui, Ui_Font_Kind kind, f32 top, f32 height)
{
    return top + 0.5f * (height - get_ui_font_size(ui, kind));
}

static void draw_rounded(Rectangle area, Color color)
{
    f32 shorter = area.width < area.height ? area.width : area.height;
    DrawRectangleRounded(area, 2.0f * UI_RADIUS / shorter, 6, color);
}

static void draw_rounded_outline(Rectangle area, Color color)
{
    f32 shorter = area.width < area.height ? area.width : area.height;
    DrawRectangleRoundedLinesEx(area, 2.0f * UI_RADIUS / shorter, 6, 1.0f, color);
}

void draw_ui_section(const Ui *ui, const char *title, f32 x, f32 *y, f32 width)
{
    draw_ui_text(ui, UI_FONT_SMALL, title, x, *y, ui->theme.text_dim);
    f32 rule_y = *y + get_ui_font_size(ui, UI_FONT_SMALL) + 6.0f;
    DrawRectangle((i32)x, (i32)rule_y, (i32)width, 1, ui->theme.border);
    *y = rule_y + 10.0f;
}

bool draw_ui_button(Ui *ui, Rectangle area, const char *label, const char *shortcut, bool enabled)
{
    const Ui_Theme *theme = &ui->theme;
    bool hover = enabled && CheckCollisionPointRec(ui->mouse, area);
    Color fill = !hover           ? theme->control
                 : ui->mouse_down ? theme->control_pressed
                                  : theme->control_hover;
    draw_rounded(area, fill);
    draw_rounded_outline(area, hover ? theme->text_disabled : theme->border);
    Color text = enabled ? theme->text : theme->text_disabled;
    draw_ui_text(ui, UI_FONT_BODY, label, area.x + 10.0f,
                 get_text_top(ui, UI_FONT_BODY, area.y, area.height), text);
    if (shortcut) {
        f32 width = measure_ui_text(ui, UI_FONT_SMALL, shortcut);
        draw_ui_text(ui, UI_FONT_SMALL, shortcut, area.x + area.width - width - 10.0f,
                     get_text_top(ui, UI_FONT_SMALL, area.y, area.height),
                     enabled ? theme->text_dim : theme->text_disabled);
    }
    return hover && ui->mouse_pressed;
}

static Ui_Animation *find_animation(Ui *ui, const void *key, f32 start)
{
    for (u32 i = 0; i < ui->animation_count; i++) {
        if (ui->animations[i].key == key) {
            return &ui->animations[i];
        }
    }
    if (ui->animation_count == UI_MAX_ANIMATIONS) {
        return NULL;
    }
    ui->animations[ui->animation_count] = Ui_Animation{key, start};
    return &ui->animations[ui->animation_count++];
}

bool draw_ui_toggle(Ui *ui, Rectangle row, const char *label, const char *shortcut, bool *value,
                    bool enabled)
{
    const Ui_Theme *theme = &ui->theme;
    bool hover = enabled && CheckCollisionPointRec(ui->mouse, row);
    bool clicked = hover && ui->mouse_pressed;
    if (clicked) {
        *value = !*value;
    }
    bool on = *value && enabled;

    Color text = !enabled ? theme->text_disabled : hover ? WHITE : theme->text;
    draw_ui_text(ui, UI_FONT_BODY, label, row.x, get_text_top(ui, UI_FONT_BODY, row.y, row.height),
                 text);

    // The switch: a pill track with a round knob that slides across.
    const f32 track_width = 34.0f;
    const f32 track_height = 18.0f;
    Rectangle track = {row.x + row.width - track_width, row.y + 0.5f * (row.height - track_height),
                       track_width, track_height};
    if (shortcut) {
        f32 width = measure_ui_text(ui, UI_FONT_SMALL, shortcut);
        draw_ui_text(ui, UI_FONT_SMALL, shortcut, track.x - width - 10.0f,
                     get_text_top(ui, UI_FONT_SMALL, row.y, row.height),
                     enabled ? theme->text_dim : theme->text_disabled);
    }
    Ui_Animation *animation = find_animation(ui, value, on ? 1.0f : 0.0f);
    f32 position = on ? 1.0f : 0.0f;
    if (animation) {
        f32 step = UI_KNOB_SPEED * ui->frame_seconds;
        animation->position = animation->position < position
                                  ? min(animation->position + step, position)
                                  : max(animation->position - step, position);
        position = animation->position;
    }
    Color accent = enabled ? theme->blue : theme->text_disabled;
    Color off_track = theme->control_pressed;
    Color track_color = ColorLerp(off_track, accent, position);
    DrawRectangleRounded(track, 1.0f, 12, track_color);
    f32 knob_radius = 0.5f * track_height - 3.0f;
    f32 knob_left = track.x + 3.0f + knob_radius;
    f32 knob_right = track.x + track.width - 3.0f - knob_radius;
    Vector2 knob = {knob_left + (knob_right - knob_left) * position, track.y + 0.5f * track_height};
    DrawCircleV(knob, knob_radius, enabled ? WHITE : theme->text_dim);
    return clicked;
}

i32 draw_ui_segmented(Ui *ui, Rectangle area, const char *const *labels, const Color *accents,
                      u32 count, u32 selected)
{
    const Ui_Theme *theme = &ui->theme;
    draw_rounded(area, theme->control);
    draw_rounded_outline(area, theme->border);
    i32 clicked = -1;
    f32 width = area.width / (f32)count;
    for (u32 i = 0; i < count; i++) {
        Rectangle part = {area.x + (f32)i * width, area.y, width, area.height};
        Rectangle inset = {part.x + 2.0f, part.y + 2.0f, part.width - 4.0f, part.height - 4.0f};
        bool hover = CheckCollisionPointRec(ui->mouse, part);
        if (i == selected) {
            draw_rounded(inset, accents[i]);
        } else if (hover) {
            draw_rounded(inset, theme->control_hover);
        }
        if (hover && ui->mouse_pressed) {
            clicked = (i32)i;
        }
        Color text = i == selected ? make_color(0x0f1115) : hover ? theme->text : theme->text_dim;
        f32 text_width = measure_ui_text(ui, UI_FONT_BOLD, labels[i]);
        draw_ui_text(ui, UI_FONT_BOLD, labels[i], part.x + 0.5f * (part.width - text_width),
                     get_text_top(ui, UI_FONT_BOLD, part.y, part.height), text);
    }
    return clicked;
}

void draw_ui_dot(f32 x, f32 y, Color color)
{
    DrawCircleV(Vector2{x, y}, 4.0f, color);
    DrawCircleV(Vector2{x, y}, 7.0f, Fade(color, 0.18f));
}
