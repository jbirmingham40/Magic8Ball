#include "Arduino.h"
#include "magic8ball.h"
#include "Gyro_QMI8658.h"
#include "lvgl.h"
#include "esp_system.h"

#define NUM_PHRASES 25

const char *responses[NUM_PHRASES]
{
    "It is\ncertain",       // 0
    "It is\ndecidedly\nso",   // 1
    "Without\na doubt",     // 2
    "Yes,\ndefinitely",   // 3
    "You\nmay rely\non it",   // 4
    "As\nI see it,\nyes",    // 5
    "Most\nlikely",        // 6
    "Yes",                // 7
    "Certainly",                    // 8
    "Signs\npoint to\nyes",         // 9
    "Reply is\nhazy,\ntry again",   // 10
    "Ask\nagain\nlater",            // 11
    "Better\nnot tell\nyou now",    // 12
    "Can't\npredict\nnow",          // 13
    "Concentrate\nand ask\nagain",  // 14
    "Don't\ncount on it",     // 15
    "My\nreply is\nno",       // 16
    "My\nsources say\nno",     // 17
    "Outlook\nnot so good",   // 18
    "Very\ndoubtful",       // 19
    "Not\nsure",            // 20
    "Very\ndoubtful",       // 21
    "No",                 // 22
    "No\nchance",           // 23
    "No\nway"              // 24
};

static lv_obj_t *s_triangle = nullptr;
static lv_obj_t *s_phrase_label = nullptr;
static lv_timer_t *s_reveal_timer = nullptr;
static bool s_promptShown = false;
static bool s_isAnimating = false;
static bool s_rngSeeded = false;
static int s_currentPhraseIndex = -1;
static int s_pendingPhraseIndex = -1;
static lv_opa_t s_scene_opa = LV_OPA_COVER;
static constexpr uint32_t kFadeOutMs = 220;
static constexpr uint32_t kHiddenPauseMs = 180;
static constexpr uint32_t kFadeInMs = 320;

static void seed_rng_once() {
    if (s_rngSeeded) {
        return;
    }

    // Use the ESP32 hardware RNG so the response order does not repeat on each reboot.
    randomSeed(static_cast<unsigned long>(esp_random()));
    s_rngSeeded = true;
}

static void set_scene_opa(void *obj, int32_t value) {
    LV_UNUSED(obj);

    s_scene_opa = static_cast<lv_opa_t>(value);

    if (s_phrase_label != nullptr) {
        lv_obj_set_style_text_opa(s_phrase_label, s_scene_opa, 0);
    }

    if (s_triangle != nullptr) {
        lv_obj_invalidate(s_triangle);
    }
}

static void show_prompt() {
    if (s_phrase_label == nullptr) {
        return;
    }

    lv_label_set_text(s_phrase_label, "Ask me,\nthen\nshake");
    set_scene_opa(nullptr, LV_OPA_COVER);
    s_promptShown = true;
    s_currentPhraseIndex = -1;
    s_pendingPhraseIndex = -1;
}

static void set_phrase(int phrase_index) {
    if (s_phrase_label == nullptr || phrase_index < 0 || phrase_index >= NUM_PHRASES) {
        return;
    }

    if (phrase_index == s_currentPhraseIndex) {
        return;
    }

    lv_label_set_text(s_phrase_label, responses[phrase_index]);
    s_currentPhraseIndex = phrase_index;
}

static void animate_scene_in() {
    if (s_triangle == nullptr) {
        return;
    }

    lv_anim_t anim;
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, s_triangle);
    lv_anim_set_values(&anim, LV_OPA_TRANSP, LV_OPA_COVER);
    lv_anim_set_time(&anim, kFadeInMs);
    lv_anim_set_exec_cb(&anim, set_scene_opa);
    lv_anim_start(&anim);
}

static void animate_scene_out() {
    if (s_triangle == nullptr) {
        return;
    }

    lv_anim_t anim;
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, s_triangle);
    lv_anim_set_values(&anim, s_scene_opa, LV_OPA_TRANSP);
    lv_anim_set_time(&anim, kFadeOutMs);
    lv_anim_set_exec_cb(&anim, set_scene_opa);
    lv_anim_start(&anim);
}

static void reveal_timer_cb(lv_timer_t *timer) {
    LV_UNUSED(timer);

    if (s_phrase_label == nullptr) {
        return;
    }

    lv_timer_del(s_reveal_timer);
    s_reveal_timer = nullptr;
    s_isAnimating = false;

    set_phrase(s_pendingPhraseIndex);
    set_scene_opa(nullptr, LV_OPA_TRANSP);
    animate_scene_in();
    Serial.printf("[Magic8Ball] Reveal animation complete, showing response %d\n", s_pendingPhraseIndex);
}

static void start_reveal_animation(int phrase_index) {
    if (s_phrase_label == nullptr || phrase_index < 0 || phrase_index >= NUM_PHRASES) {
        return;
    }

    s_pendingPhraseIndex = phrase_index;
    s_isAnimating = true;
    s_currentPhraseIndex = -1;

    if (s_reveal_timer != nullptr) {
        lv_timer_del(s_reveal_timer);
        s_reveal_timer = nullptr;
    }

    animate_scene_out();
    s_reveal_timer = lv_timer_create(reveal_timer_cb, kFadeOutMs + kHiddenPauseMs, nullptr);
}

void Magic8Ball_Reset(void) {
    s_isAnimating = false;
    s_currentPhraseIndex = -1;
    s_pendingPhraseIndex = -1;

    if (s_reveal_timer != nullptr) {
        lv_timer_del(s_reveal_timer);
        s_reveal_timer = nullptr;
    }

    if (s_phrase_label != nullptr) {
        show_prompt();
    } else {
        s_promptShown = false;
    }
}


// LVGL draw event callback — called every time LVGL redraws the triangle widget.
static void draw_triangle_cb(lv_event_t *e) {
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);

    // Equilateral triangle, pointing upward, centered on the 412×412 display.
    // Side ≈ 300 px, height ≈ 260 px.
    lv_point_t pts[3] = {
        {206,  33},   // top
        { 56, 293},   // bottom-left
        {356, 293},   // bottom-right
    };

    lv_draw_rect_dsc_t dsc;
    lv_draw_rect_dsc_init(&dsc);
    dsc.bg_color    = lv_color_make(0x00, 0x00, 0xFF);  // bright blue
    dsc.bg_opa      = s_scene_opa;
    dsc.border_width = 0;
    dsc.radius      = 0;

    lv_draw_polygon(draw_ctx, &dsc, pts, 3);
}

void Magic8Ball_Init(void) {
    seed_rng_once();

    lv_obj_t *scr = lv_scr_act();

    // Black background — the 8-ball sphere.
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    if (s_triangle == nullptr) {
        // Full-screen transparent container that owns the triangle draw callback.
        // Using a draw callback avoids allocating a 412×412 canvas buffer.
        s_triangle = lv_obj_create(scr);
        lv_obj_set_size(s_triangle, LV_PCT(100), LV_PCT(100));
        lv_obj_align(s_triangle, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_opa(s_triangle, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(s_triangle, 0, 0);
        lv_obj_set_style_pad_all(s_triangle, 0, 0);
        lv_obj_clear_flag(s_triangle, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(s_triangle, draw_triangle_cb, LV_EVENT_DRAW_MAIN_END, NULL);
    }

    if (s_phrase_label == nullptr) {
        s_phrase_label = lv_label_create(s_triangle);
        lv_obj_set_width(s_phrase_label, 180);
        lv_label_set_long_mode(s_phrase_label, LV_LABEL_LONG_CLIP);
        lv_obj_set_style_text_color(s_phrase_label, lv_color_white(), 0);
        lv_obj_set_style_text_align(s_phrase_label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(s_phrase_label, &lv_font_montserrat_22, 0);
        lv_obj_set_style_text_line_space(s_phrase_label, 15, 0);  // Line spacing
        lv_obj_align(s_phrase_label, LV_ALIGN_CENTER, 0, 0);
    }

    if (!s_promptShown) {
        show_prompt();
    }

    if (QMI8658_ConsumeShakeEvent()) {
        int next_phrase = random(NUM_PHRASES);
        if (!s_isAnimating) {
            start_reveal_animation(next_phrase);
            Serial.printf("[Magic8Ball] Shake detected, animating response %d\n", next_phrase);
        }
    }
}
